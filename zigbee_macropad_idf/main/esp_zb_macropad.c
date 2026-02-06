#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_zb_macropad.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define TAG                 "MACROPAD"

// Set to 1 to enable external antenna (XIAO ESP32C6), 0 for internal
#define USE_EXTERNAL_ANTENNA 1

/* --- PINS --------------------------------------------------------------- */
#define ROWS 4
#define COLS 4
#define BTN_COUNT (ROWS * COLS)

// Optional extra input (not part of the key matrix): rotary encoder switch.
// IMPORTANT: Do NOT reuse a matrix ROW/COL pin for this, otherwise it will
// behave like multiple keys being pressed (or never debounce correctly).
// Wire: ENC_SW_GPIO -> switch -> GND, and keep pull-up enabled.
#define ENC_SW_GPIO          GPIO_NUM_22
#define ENC_BUTTON_ID        (BTN_COUNT)   // virtual button id: 16

// Rotary encoder A/B (quadrature) inputs.
// Defaults match common XIAO ESP32-C6 wiring (D5=GPIO23, D6=GPIO16).
#define ENC_A_GPIO           GPIO_NUM_23  // D5
#define ENC_B_GPIO           GPIO_NUM_16  // D6

// Most EC11-style encoders produce 4 edge transitions per detent.
#define ENC_STEPS_PER_DETENT 4

// Throttle how often we publish Zigbee rotate events.
#define ENC_REPORT_INTERVAL_MS 60

// Row input pins
static const gpio_num_t ROW_PINS[ROWS] = {
    GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_4,
}; //D0, D1, D2, MTMS

//Any available Pins
static const gpio_num_t COL_PINS[COLS] = {
    GPIO_NUM_17, GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_18,
}; //D10, D9, D8, D7

// Native physical button on board can still be connected with external button
#define BOOT_BUTTON_GPIO     GPIO_NUM_9

/* --- Timing (ms) for local keypad -------------------------------------- */
#define BTN_POLL_INTERVAL_MS   10
#define DEBOUNCE_MS        15
#define DOUBLE_CLICK_MS   400
#define HOLD_PRESS_MS    1000
// Encoder switch: use ultra-long press to enter pairing mode.
#define ENC_ULTRA_PRESS_MS 8000

/* --- Endpoint and clusters -------------------------------------- */
#define MACROPAD_ENDPOINT            0x01

/* Custom cluster used to report button events to Z2M */
#define MACROPAD_CLUSTER_ID          0xFC00
#define MACROPAD_CMD_BUTTON_EVENT    0x00
// Generic rotary event: payload = { direction, steps }
// direction: 0 = left/ccw, 1 = right/cw
// steps: number of detents since last report (1..255)
#define MACROPAD_CMD_ENCODER_ROTATE  0x01
/* Use all channels or restrict as you like */
#define ESP_ZB_PRIMARY_CHANNEL_MASK  ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* --- Button state machine ----------------------------------------------- */
typedef struct {
    bool raw;
    bool stable;
    bool prev_stable;
    uint64_t last_change_us;
    uint64_t press_start_us;
    uint64_t last_release_us;
    bool hold_fired;
} btn_state_t;

static QueueHandle_t s_boot_evt_q = NULL;
static QueueHandle_t s_enc_evt_q  = NULL;

static btn_state_t g_btn[BTN_COUNT];
static btn_state_t g_enc_btn;

/* --- Zigbee state ------------------------------------------------------- */
static bool g_is_joined     = false;
static bool g_zb_ready      = false;

/* --- Helpers ------------------------------------------------------------ */
static inline uint64_t now_us(void) { return esp_timer_get_time(); }

typedef enum { ACT_NONE, ACT_SINGLE, ACT_DOUBLE, ACT_HOLD } action_t;

/* --- Forward declarations  ------------------------------------------------------ */
static const char* action_str(action_t a);
static void start_network_steering(uint8_t param);
static void zb_reset_and_steer_cb(void);

/* Prototypes */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

/* Public helper to send a button event (call this from your key matrix code) */
void macropad_send_button_event(uint8_t button_id, action_t action);

static void macropad_send_encoder_rotate(bool clockwise, uint8_t steps);

/* ======================================================================= */
/*                          INIT GPIO                                      */
/* ======================================================================= */
static void matrix_gpio_init(void)
{
    // Rows: inputs with pull-up
    for (int r = 0; r < ROWS; ++r) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << ROW_PINS[r],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }

    // Columns: outputs, start HIGH
    for (int c = 0; c < COLS; ++c) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << COL_PINS[c],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(COL_PINS[c], 1); // idle high
    }
}

/* ======================================================================= */
/*                   FACTORY RESET + COMMISSION (ZB CTX)                   */
/* ======================================================================= */
static void start_network_steering(uint8_t param)
{
    (void)param;
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void zb_reset_and_steer_cb(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing Zigbee NVS and restarting commissioning...");
    g_is_joined = false;

    esp_zb_factory_reset();

    /* Start network steering shortly after */
    esp_zb_scheduler_alarm(start_network_steering, 0, 1500);
}

/* ======================================================================= */
/*                      BOOT BUTTON (ISR + QUEUE)                          */
/* ======================================================================= */
static void IRAM_ATTR boot_button_isr(void *arg) {
    uint32_t dummy = 1;
    xQueueSendFromISR(s_boot_evt_q, &dummy, NULL);
}

static void IRAM_ATTR encoder_isr(void *arg)
{
    (void)arg;
    uint32_t dummy = 1;
    if (s_enc_evt_q) {
        xQueueSendFromISR(s_enc_evt_q, &dummy, NULL);
    }
}

static void boot_button_task(void *arg) {
    uint32_t dummy;
    while (1) {
        if (xQueueReceive(s_boot_evt_q, &dummy, portMAX_DELAY)) {
            if (g_zb_ready) {
                esp_zb_scheduler_alarm((esp_zb_callback_t)zb_reset_and_steer_cb, 0, 0);
            } else {
                ESP_LOGW(TAG, "Zigbee not ready; ignoring button press");
            }
        }
    }
}

/* ======================================================================= */
/*                     ROTARY ENCODER (A/B QUADRATURE)                     */
/* ======================================================================= */
static int8_t enc_quadrature_delta(uint8_t prev, uint8_t curr)
{
    // prev/curr are 2-bit states: (A<<1)|B
    // Table indexed by (prev<<2)|curr.
    // Values: -1, 0, +1 per valid transition; invalid transitions -> 0.
    static const int8_t table[16] = {
        0,  -1,  +1,  0,
        +1,  0,   0, -1,
        -1,  0,   0, +1,
        0,  +1,  -1,  0,
    };
    return table[((prev & 0x3) << 2) | (curr & 0x3)];
}

static uint8_t enc_read_state(void)
{
    const uint8_t a = (gpio_get_level(ENC_A_GPIO) != 0) ? 1 : 0;
    const uint8_t b = (gpio_get_level(ENC_B_GPIO) != 0) ? 1 : 0;
    return (uint8_t)((a << 1) | b);
}

static void encoder_task(void *arg)
{
    (void)arg;

    const uint64_t report_interval_us = ENC_REPORT_INTERVAL_MS * 1000ULL;

    uint32_t dummy;
    uint8_t prev = enc_read_state();
    int32_t edge_acc = 0;
    int32_t detent_acc = 0;
    uint64_t last_report_us = now_us();

    while (true) {
        // Wait for edges; also periodically flush accumulated detents.
        const BaseType_t got = xQueueReceive(s_enc_evt_q, &dummy, pdMS_TO_TICKS(ENC_REPORT_INTERVAL_MS));
        const uint64_t now = now_us();

        if (got) {
            const uint8_t curr = enc_read_state();
            const int8_t d = enc_quadrature_delta(prev, curr);
            prev = curr;

            if (d != 0) {
                edge_acc += d;

                // Convert edge counts to detents (usually 4 edges per detent).
                if (edge_acc >= ENC_STEPS_PER_DETENT || edge_acc <= -ENC_STEPS_PER_DETENT) {
                    const int32_t detents = edge_acc / ENC_STEPS_PER_DETENT;
                    edge_acc = edge_acc % ENC_STEPS_PER_DETENT;
                    detent_acc += detents;
                }
            }
        }

        if (detent_acc != 0 && (now - last_report_us) >= report_interval_us) {
            // Only publish when joined; keep the input responsive regardless.
            if (g_is_joined) {
                const bool clockwise = (detent_acc > 0);
                uint32_t steps = (uint32_t)(clockwise ? detent_acc : -detent_acc);
                if (steps == 0) {
                    steps = 1;
                }
                if (steps > 255) {
                    steps = 255;
                }

                macropad_send_encoder_rotate(clockwise, (uint8_t)steps);
            }

            detent_acc = 0;
            last_report_us = now;
        }
    }
}

/* ======================================================================= */
/*                     LOCAL FEEDBACK (color fixed)                        */
/* ======================================================================= */
static const char* action_str(action_t a) {
    return (a==ACT_SINGLE) ? "single" : (a==ACT_DOUBLE) ? "double" : "hold";
}

static void on_button_action(uint8_t index, action_t act) {
    ESP_LOGI(TAG, "Button %u -> %s", (unsigned)index, action_str(act));
    macropad_send_button_event(index, act);
}

/* ======================================================================= */
/*                        BUTTONS TASK                                     */
/* ======================================================================= */
// Fill raw_states[BTN_COUNT] with true if pressed, false otherwise
static void matrix_scan(bool raw_states[BTN_COUNT])
{
    // Clear all
    for (int i = 0; i < BTN_COUNT; ++i) {
        raw_states[i] = false;
    }

    for (int c = 0; c < COLS; ++c) {
        // Set all columns HIGH
        for (int cc = 0; cc < COLS; ++cc) {
            gpio_set_level(COL_PINS[cc], 1);
        }

        // Drive current column LOW (active)
        gpio_set_level(COL_PINS[c], 0);

        // Small settle delay (a few microseconds is enough)
        esp_rom_delay_us(5);

        for (int r = 0; r < ROWS; ++r) {
            int level = gpio_get_level(ROW_PINS[r]); // 0 = pressed (active low)
            bool pressed = (level == 0);

            int idx = r * COLS + c;  // mapping row/col -> button index
            raw_states[idx] = pressed;
        }
    }

    // Return columns to idle HIGH (optional but nice)
    for (int c = 0; c < COLS; ++c) {
        gpio_set_level(COL_PINS[c], 1);
    }
}

static void btn_state_reset(btn_state_t *b)
{
    memset(b, 0, sizeof(*b));
}

static void process_button_state(uint8_t button_id, btn_state_t *b, bool raw,
                                 uint64_t now, uint64_t debounce_us,
                                 uint64_t double_click_us, uint64_t long_press_us)
{
    // Debounce transition
    if (raw != b->stable && now - b->last_change_us > debounce_us) {
        b->prev_stable = b->stable;
        b->stable = raw;
        b->last_change_us = now;

        if (b->stable) {
            // pressed
            b->press_start_us = now;
            b->hold_fired = false;
        } else {
            // released
            if (!b->hold_fired) {
                if (b->last_release_us && (now - b->last_release_us < double_click_us)) {
                    on_button_action(button_id, ACT_DOUBLE);
                    b->last_release_us = 0;
                } else {
                    b->last_release_us = now;
                }
            }
        }
    }

    // long press detection
    if (b->stable && !b->hold_fired && now - b->press_start_us > long_press_us) {
        b->hold_fired = true;
        on_button_action(button_id, ACT_HOLD);
        b->last_release_us = 0;
    }

    // single click confirmation (timeout expired)
    if (!b->stable && b->last_release_us && now - b->last_release_us > double_click_us) {
        on_button_action(button_id, ACT_SINGLE);
        b->last_release_us = 0;
    }
}

static void process_encoder_switch(btn_state_t *b, bool raw,
                                  uint64_t now, uint64_t debounce_us,
                                  uint64_t double_click_us, uint64_t ultra_long_us)
{
    // Debounce transition (same as normal buttons)
    if (raw != b->stable && now - b->last_change_us > debounce_us) {
        b->prev_stable = b->stable;
        b->stable = raw;
        b->last_change_us = now;

        if (b->stable) {
            // pressed
            b->press_start_us = now;
            b->hold_fired = false; // reused as "ultra fired"
        } else {
            // released
            if (!b->hold_fired) {
                if (b->last_release_us && (now - b->last_release_us < double_click_us)) {
                    // Only emit click actions when joined (otherwise there's no coordinator to report to).
                    if (g_is_joined) {
                        on_button_action(ENC_BUTTON_ID, ACT_DOUBLE);
                    }
                    b->last_release_us = 0;
                } else {
                    b->last_release_us = now;
                }
            } else {
                // ultra-long already triggered; swallow clicks on release
                b->last_release_us = 0;
            }
        }
    }

    // Ultra-long press -> enter pairing (factory reset + steering)
    if (b->stable && !b->hold_fired && now - b->press_start_us > ultra_long_us) {
        b->hold_fired = true;
        b->last_release_us = 0;

        ESP_LOGW(TAG, "Encoder ultra-long press -> pairing");
        if (g_zb_ready) {
            esp_zb_scheduler_alarm((esp_zb_callback_t)zb_reset_and_steer_cb, 0, 0);
        } else {
            ESP_LOGW(TAG, "Zigbee not ready; ignoring pairing request");
        }
    }

    // Single click confirmation (timeout expired)
    if (!b->stable && b->last_release_us && now - b->last_release_us > double_click_us) {
        // Only emit click actions when joined.
        if (g_is_joined) {
            on_button_action(ENC_BUTTON_ID, ACT_SINGLE);
        }
        b->last_release_us = 0;
    }
}

static void button_task(void *arg)
{
    
    const uint64_t debounce_us      = DEBOUNCE_MS * 1000ULL;
    const uint64_t double_click_us  = DOUBLE_CLICK_MS * 1000ULL;
    const uint64_t long_press_us    = HOLD_PRESS_MS * 1000ULL;
    const uint64_t enc_ultra_us     = ENC_ULTRA_PRESS_MS * 1000ULL;

    bool raw_states[BTN_COUNT];
    while (true) {
        uint64_t now = now_us();

        // Always process encoder switch so ultra-long press can trigger pairing even
        // when the device is factory-new / not joined yet.
        bool enc_raw = (gpio_get_level(ENC_SW_GPIO) == 0);
        process_encoder_switch(&g_enc_btn, enc_raw, now,
                               debounce_us, double_click_us, enc_ultra_us);

        // Skip scanning matrix buttons until joined (avoids spamming logs/logic when not connected).
        if(!g_is_joined)
        {
            vTaskDelay(pdMS_TO_TICKS(BTN_POLL_INTERVAL_MS));
            continue;
        }

        // 1. Scan whole matrix once -> raw_states[]
        matrix_scan(raw_states);
        
        // 2. Run your existing state machine per logical button
        for (int i = 0; i < BTN_COUNT; ++i) {
            process_button_state((uint8_t)i, &g_btn[i], raw_states[i], now,
                                 debounce_us, double_click_us, long_press_us);
        }

        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_INTERVAL_MS));
    }
}

/* ======================================================================= */
/*                      ZIGBEE SIGNAL HANDLER                              */
/* ======================================================================= */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p_sg_p    = sig->p_app_signal;
    esp_err_t err_status = sig->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        // ESP_FAIL here only means "no production config found" (normal for non-manufacturing builds).
        // Don't treat it as fatal.
        ESP_LOGI(TAG, "Production config ready status: %s", esp_err_to_name(err_status));
        break;

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (err_status != ESP_OK) {
            ESP_LOGE(TAG, "Zigbee stack startup failed: %s", esp_err_to_name(err_status));
            break;
        }
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            bool is_fn = esp_zb_bdb_is_factory_new();
            g_is_joined = !is_fn;
            ESP_LOGI(TAG, "Device %s factory new", is_fn ? "is" : "is not");

            if (is_fn) {
                ESP_LOGI(TAG, "Starting network steering...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            g_is_joined = true;
            ESP_LOGI(TAG, "Joined network successfully");
        } else {
            g_is_joined = false;
            ESP_LOGI(TAG, "Steering failed → retry");
            /* Retry steering after 1 second */
            esp_zb_scheduler_alarm(start_network_steering, 0, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        g_is_joined = false;
        ESP_LOGW(TAG, "Left network → rejoining");
        esp_zb_scheduler_alarm(start_network_steering, 0, 1500);
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x) status: %s",
                 esp_zb_zdo_signal_to_string(sig_type),
                 sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* ======================================================================= */
/*                       ZIGBEE STACK TASK (manual Level Control)          */
/* ======================================================================= */
static void esp_zb_task(void *pv)
{
    /*---------------------------------------------------------------
     * Initialize Zigbee stack (role must match sdkconfig)
     * - CONFIG_ZB_ZED=y  -> End Device
     * - CONFIG_ZB_ZCZR=y -> Coordinator/Router
     *-------------------------------------------------------------*/
#if CONFIG_ZB_ZED
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
#else
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
#endif
    esp_zb_init(&zb_nwk_cfg);

    /*---------------------------------------------------------------
     * BASIC CLUSTER (mandatory)
     *-------------------------------------------------------------*/
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);

    static const char manufacturer_name[] = ESP_MANUFACTURER_NAME;
    static const char model_identifier[]  = ESP_MODEL_IDENTIFIER;

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(basic_cluster,
                                            ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                            (void *)manufacturer_name));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(basic_cluster,
                                            ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                            ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                            (void *)model_identifier));

    /*---------------------------------------------------------------
     * IDENTIFY CLUSTER (mandatory)
     *-------------------------------------------------------------*/
    esp_zb_identify_cluster_cfg_t identify_cfg = {.identify_time = 0};
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);

    /*---------------------------------------------------------------
     * CUSTOM MACROPAD CLUSTER 
     *-------------------------------------------------------------*/
    esp_zb_attribute_list_t *macropad_cluster =
    esp_zb_zcl_attr_list_create(MACROPAD_CLUSTER_ID);

    /*---------------------------------------------------------------
     * CLUSTER LIST (Basic + Identify + Macropad)
     *-------------------------------------------------------------*/
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(cluster_list ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list,
                                                          basic_cluster,
                                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list,
                                                             identify_cluster,
                                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster( cluster_list,
                                                            macropad_cluster,
                                                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /*---------------------------------------------------------------
     * ENDPOINT CONFIGURATION
     *-------------------------------------------------------------*/
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint           = MACROPAD_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(ep_list ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg));

    /*---------------------------------------------------------------
     * REGISTER DEVICE + CALLBACKS
     *-------------------------------------------------------------*/
    esp_zb_device_register(ep_list);

    /* 3. Register Zigbee core action handler (for attribute writes, custom commands, etc.) */
    esp_zb_core_action_handler_register(zb_action_handler);
    
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    

    g_zb_ready = true;
    esp_zb_stack_main_loop();
}

/* ======================================================================= */
/*                       ZIGBEE ATTRIUBUTE HANDLER                         */
/* ======================================================================= */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty ZCL set_attr message");

    if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "set_attr: error status=%d", message->info.status);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t  endpoint   = message->info.dst_endpoint;
    uint16_t cluster_id = message->info.cluster;
    uint16_t attr_id    = message->attribute.id;

    ESP_LOGI(TAG,
             "set_attr: ep=%u cluster=0x%04X attr=0x%04X size=%d",
             endpoint,
             cluster_id,
             attr_id,
             message->attribute.data.size);

    switch (cluster_id) {
    case ESP_ZB_ZCL_CLUSTER_ID_BASIC:
        /* Handle writes to Basic cluster if needed (e.g. IdentifyTime) */
        break;

    case MACROPAD_CLUSTER_ID:
        /* No writable attributes (button events are sent as custom commands). */
        break;
    
    default:
        /* Unknown or unhandled cluster */
        break;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) 
{
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler(
            (const esp_zb_zcl_set_attr_value_message_t *)message);
        break;

    /* You can add more cases later:
     *  - ESP_ZB_CORE_REPORT_ATTR_CB_ID
     *  - ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID
     *  etc.
     */
    default:
        ESP_LOGW(TAG, "Unhandled Zigbee action 0x%x", callback_id);
        break;
    }

    return ret;
}

void macropad_send_button_event(uint8_t button_id, action_t action)
{
    uint8_t payload[2] = { button_id, (uint8_t)action};

    esp_zb_zcl_custom_cluster_cmd_req_t req = {0};

    req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;  // coordinator
    req.zcl_basic_cmd.dst_endpoint          = MACROPAD_ENDPOINT;
    req.zcl_basic_cmd.src_endpoint          = MACROPAD_ENDPOINT;

    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.cluster_id   = MACROPAD_CLUSTER_ID;
    req.profile_id   = ESP_ZB_AF_HA_PROFILE_ID;
    req.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    req.custom_cmd_id = MACROPAD_CMD_BUTTON_EVENT;

    req.data.type  = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size  = sizeof(payload);
    req.data.value = payload;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();

    //ESP_EARLY_LOGI(TAG, "Sent button event: button=%u action=%u", button_id, action);
}

static void macropad_send_encoder_rotate(bool clockwise, uint8_t steps)
{
    // payload: {direction, steps}
    // Logic inversion: if clockwise (true) -> send 0 (left/ccw)
    //                 if !clockwise (false) -> send 1 (right/cw)
    const uint8_t payload[2] = { (uint8_t)(clockwise ? 0 : 1), steps };

    esp_zb_zcl_custom_cluster_cmd_req_t req = {0};

    req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;  // coordinator
    req.zcl_basic_cmd.dst_endpoint          = MACROPAD_ENDPOINT;
    req.zcl_basic_cmd.src_endpoint          = MACROPAD_ENDPOINT;

    req.address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.cluster_id    = MACROPAD_CLUSTER_ID;
    req.profile_id    = ESP_ZB_AF_HA_PROFILE_ID;
    req.direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    req.custom_cmd_id = MACROPAD_CMD_ENCODER_ROTATE;

    req.data.type  = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size  = sizeof(payload);
    req.data.value = (void *)payload;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
}

/* ======================================================================= */
/*                                MAIN                                     */
/* ======================================================================= */
void app_main(void)
{
    // --- Enable EXTERNAL ANTENNA (XIAO ESP32C6) ---
    // See: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#hardware-overview
#if USE_EXTERNAL_ANTENNA
    {
        gpio_config_t ant_conf = {
            .pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_14),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&ant_conf);
        
        gpio_set_level(GPIO_NUM_3, 0);   // Enable RF switch (Low)
        vTaskDelay(pdMS_TO_TICKS(50));   // Short delay for stability
        gpio_set_level(GPIO_NUM_14, 1);  // Select External Antenna (High)
        
        ESP_LOGI(TAG, "External antenna enabled (GPIO3=0, GPIO14=1)");
    }
#else
    ESP_LOGI(TAG, "Using internal antenna");
#endif

    matrix_gpio_init();
    // Initialise button state array
    for (int i = 0; i < BTN_COUNT; ++i) {
        btn_state_reset(&g_btn[i]);
    }
    btn_state_reset(&g_enc_btn);
    nvs_flash_init();

    ESP_LOGI(TAG, "Starting 16-button macropad");

    // --- Configure encoder switch GPIO (optional) ---
    {
        gpio_config_t encio = {
            .pin_bit_mask = 1ULL << ENC_SW_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&encio));
        ESP_LOGI(TAG, "Encoder SW on GPIO %d (button id %d)", (int)ENC_SW_GPIO, (int)ENC_BUTTON_ID);
    }

    // --- Configure rotary encoder A/B GPIOs (quadrature) ---
    {
        gpio_config_t encab = {
            .pin_bit_mask = (1ULL << ENC_A_GPIO) | (1ULL << ENC_B_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&encab));
        ESP_LOGI(TAG, "Encoder A/B on GPIO %d/%d", (int)ENC_A_GPIO, (int)ENC_B_GPIO);
    }
    
    // --- Configure BOOT button input ---
    gpio_config_t btnio = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&btnio));

    // --- Create queue ---
    s_boot_evt_q = xQueueCreate(4, sizeof(uint32_t));
    s_enc_evt_q  = xQueueCreate(16, sizeof(uint32_t));

    // --- Install ISR service ONCE ---
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, boot_button_isr, NULL);
    gpio_isr_handler_add(ENC_A_GPIO, encoder_isr, NULL);
    gpio_isr_handler_add(ENC_B_GPIO, encoder_isr, NULL);

    // --- Launch tasks ---
    xTaskCreate(button_task, "button_task", 4096, NULL, 1, NULL);
    xTaskCreate(boot_button_task, "boot_btn", 2048, NULL, 1, NULL);
    xTaskCreate(encoder_task, "encoder", 3072, NULL, 2, NULL);

    /* --- ZIGBEE --------------------------------------------------------- */
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "ZB_main", 12288, NULL, 5, NULL);        
    
    ESP_LOGI(TAG, "Ready.");
}
