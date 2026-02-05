// nanoESP32-C6-N8 — 16-button Zigbee remote with pairing support
#ifndef ZIGBEE_MODE_ED 
#error "Zigbee end device mode is not selected in Tools->Zigbee mode" 
#endif

#include <Arduino.h>
#include <Zigbee.h>

// ----------------------- Hardware configuration -----------------------
#define BUTTON_COUNT 16
const uint8_t buttonPins[BUTTON_COUNT] = {
  1, 2, 3, 4, 5, 6, 7, 10,
  11, 12, 13, 18, 19, 20, 21, 22
};

#define BOOT_BUTTON 9

// ----------------------- Rotary encoder (optional) ---------------------
// Board silkscreen: SW=D4, DT=D5, CLK=D6
// Note: these may overlap with entries in buttonPins[] on some wiring.
// If they do, those button indices will be ignored to avoid ghost actions.
// IMPORTANT: On Arduino, the numeric value you pass to pinMode/digitalRead
// must match your board's GPIO mapping. For your board:
//   D4 = GPIO22, D5 = GPIO23, D6 = GPIO16
constexpr uint8_t ENC_SW_PIN  = 22; // D4
constexpr uint8_t ENC_DT_PIN  = 23; // D5
constexpr uint8_t ENC_CLK_PIN = 16; // D6

constexpr bool ENC_LOG_TRANSITIONS = true;
// Extra debug: prints raw levels when they change (helps confirm wiring/pin mapping).
constexpr bool ENC_DEBUG_RAW_LEVELS = true;

// Virtual "button index" for the encoder switch.
// This is not part of buttonPins[]; it is emitted by the encoder handler.
constexpr uint8_t BUTTON_ENC = BUTTON_COUNT;

// ----------------------- Timing and thresholds ------------------------
constexpr unsigned long DEBOUNCE_MS = 20;
constexpr unsigned long DOUBLE_MS = 400;
constexpr unsigned long LONG_MS = 1000;
constexpr unsigned long ULTRA_LONG_MS = 8000;

// ----------------------- Action encoding colors ------------------------
// These RGB values are used only to encode actions over Zigbee.
// No on-board/external LED is driven.

struct RGB { uint8_t r, g, b; };
const RGB COLOR_OFF    = {0, 0, 0};
const RGB COLOR_GREEN  = {0, 255, 0};
const RGB COLOR_BLUE   = {0, 0, 255};
const RGB COLOR_YELLOW = {255, 255, 0};
const RGB COLOR_RED    = {255, 0, 0};

// ----------------------- Button logic ---------------------------------
bool lastRawState[BUTTON_COUNT];
bool stableState[BUTTON_COUNT];
unsigned long lastStableTime[BUTTON_COUNT];
unsigned long pressStartTime[BUTTON_COUNT];
unsigned long lastReleaseTime[BUTTON_COUNT];
bool longAlreadyTriggered[BUTTON_COUNT];

// Forward declarations used by encoder handler
extern bool pairingActive;
void startPairing();

static inline bool isReservedPin(uint8_t pin) {
  return (pin == ENC_SW_PIN) || (pin == ENC_DT_PIN) || (pin == ENC_CLK_PIN);
}

// Rotary encoder switch state (single/double clicks + ultra-long press)
bool encSw_lastRaw = false;
bool encSw_stable = false;
unsigned long encSw_lastChangeMs = 0;
unsigned long encSw_pressStartMs = 0;
bool encSw_ultraTriggered = false;
unsigned long encSw_lastReleaseMs = 0;

void handleButtons(void (*onAction)(uint8_t, const char *)) {
  unsigned long now = millis();
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    if (isReservedPin(buttonPins[i])) {
      continue;
    }
    bool raw = (digitalRead(buttonPins[i]) == LOW);
    if (raw != lastRawState[i]) {
      lastRawState[i] = raw;
      lastStableTime[i] = now;
    }

    if ((now - lastStableTime[i]) >= DEBOUNCE_MS) {
      if (raw != stableState[i]) {
        stableState[i] = raw;
        if (stableState[i]) {
          pressStartTime[i] = now;
          longAlreadyTriggered[i] = false;
        } else {
          unsigned long pressDur = now - pressStartTime[i];
          if (!longAlreadyTriggered[i]) {
            if (lastReleaseTime[i] && (now - lastReleaseTime[i]) <= DOUBLE_MS) {
              onAction(i, "double");
              lastReleaseTime[i] = 0;
            } else {
              lastReleaseTime[i] = now;
            }
          } else {
            lastReleaseTime[i] = 0;
          }
        }
      }
    }

    if (stableState[i] && !longAlreadyTriggered[i] &&
        (now - pressStartTime[i]) >= LONG_MS) {
      longAlreadyTriggered[i] = true;
      onAction(i, "long");
      lastReleaseTime[i] = 0;
    }

    if (lastReleaseTime[i] && ((now - lastReleaseTime[i]) > DOUBLE_MS)) {
      onAction(i, "single");
      lastReleaseTime[i] = 0;
    }
  }
}

void handleEncoderSwitch(void (*onAction)(uint8_t, const char *)) {
  unsigned long now = millis();
  bool rawPressed = (digitalRead(ENC_SW_PIN) == LOW);

  if (rawPressed != encSw_lastRaw) {
    encSw_lastRaw = rawPressed;
    encSw_lastChangeMs = now;
  }

  if ((now - encSw_lastChangeMs) >= DEBOUNCE_MS) {
    if (rawPressed != encSw_stable) {
      encSw_stable = rawPressed;
      if (encSw_stable) {
        if (ENC_LOG_TRANSITIONS) {
          Serial.println("ENC press");
        }
        encSw_pressStartMs = now;
        encSw_ultraTriggered = false;
      } else {
        if (ENC_LOG_TRANSITIONS) {
          Serial.println("ENC release");
        }
        // Release: treat as click only if ultra-long press didn't trigger
        if (!encSw_ultraTriggered) {
          if (encSw_lastReleaseMs && (now - encSw_lastReleaseMs) <= DOUBLE_MS) {
            onAction(BUTTON_ENC, "double");
            encSw_lastReleaseMs = 0;
          } else {
            encSw_lastReleaseMs = now;
          }
        } else {
          encSw_lastReleaseMs = 0;
        }

        encSw_pressStartMs = 0;
        encSw_ultraTriggered = false;
      }
    }
  }

  // Ultra-long press (no standard long-press mapping for encoder)
  if (encSw_stable && !encSw_ultraTriggered && !pairingActive &&
      encSw_pressStartMs && (now - encSw_pressStartMs) >= ULTRA_LONG_MS) {
    encSw_ultraTriggered = true;
    encSw_lastReleaseMs = 0;
    startPairing();
  }

  // Single click is emitted after the double-click window expires
  if (encSw_lastReleaseMs && ((now - encSw_lastReleaseMs) > DOUBLE_MS)) {
    onAction(BUTTON_ENC, "single");
    encSw_lastReleaseMs = 0;
  }
}

// ----------------------- Zigbee endpoint ------------------------------
#define ZIGBEE_ENDPOINT 10
ZigbeeColorDimmableLight zbLight(ZIGBEE_ENDPOINT);

void emitZigbeeAction(uint8_t index, uint8_t actionCode, const RGB &color) {
  uint8_t level = (index) * 6 + actionCode;
  zbLight.setLight(true, level, color.r, color.g, color.b);
  delay(60);
  zbLight.setLight(false, 0, 0, 0, 0);
}

void onButtonAction(uint8_t index, const char *action) {
  if (index == BUTTON_ENC) {
    Serial.printf("ENC -> %s\n", action);
  } else {
    Serial.printf("Button %u -> %s\n", index + 1, action);
  }

  RGB color = COLOR_OFF;
  uint8_t actionCode = 0;
  if (strcmp(action, "single") == 0) { color = COLOR_GREEN; actionCode = 1; }
  else if (strcmp(action, "double") == 0) { color = COLOR_BLUE; actionCode = 2; }
  else if (strcmp(action, "long") == 0) { color = COLOR_YELLOW; actionCode = 3; }

  emitZigbeeAction(index, actionCode, color);
}

// ----------------------- Boot button pairing --------------------------
bool pairingActive = false;

void startPairing() {
  pairingActive = true;
  Serial.println("[ZB] Factory reset + Zigbee pairing...");
  Zigbee.factoryReset();   // starts join mode
}

// ----------------------- Setup & loop ---------------------------------
void setup() {
  Serial.begin(115200);

  if (ENC_DEBUG_RAW_LEVELS) {
    Serial.printf("ENC pins (Arduino numbers): SW=%u DT=%u CLK=%u\n",
                  (unsigned)ENC_SW_PIN, (unsigned)ENC_DT_PIN, (unsigned)ENC_CLK_PIN);
  }

  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    if (!isReservedPin(buttonPins[i])) {
      pinMode(buttonPins[i], INPUT_PULLUP);
    }
    stableState[i] = false;
    lastRawState[i] = (!isReservedPin(buttonPins[i])) && (digitalRead(buttonPins[i]) == LOW);
    lastStableTime[i] = millis();
  }
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  encSw_lastRaw = (digitalRead(ENC_SW_PIN) == LOW);
  encSw_stable = encSw_lastRaw;
  encSw_lastChangeMs = millis();

  if (ENC_DEBUG_RAW_LEVELS) {
    Serial.printf("ENC initial SW level: %s\n", encSw_lastRaw ? "LOW(pressed?)" : "HIGH(released?)");
  }

  zbLight.setManufacturerAndModel("Custom", "ZB16BtnRemote");
  Zigbee.addEndpoint(&zbLight);

  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    delay(2000);
    ESP.restart();
  }

  Serial.println("[ZB] Starting...");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\n[ZB] Connected.");
}

void loop() {
  if (ENC_DEBUG_RAW_LEVELS) {
    static int lastEncLevel = -1;
    const int level = digitalRead(ENC_SW_PIN);
    if (level != lastEncLevel) {
      lastEncLevel = level;
      Serial.printf("ENC_SW_PIN=%u level=%s\n", (unsigned)ENC_SW_PIN, level == LOW ? "LOW" : "HIGH");
    }
  }

  handleButtons(onButtonAction);
  handleEncoderSwitch(onButtonAction);

  if (digitalRead(BOOT_BUTTON) == LOW && !pairingActive) {
    delay(100);
    if (digitalRead(BOOT_BUTTON) == LOW) {
      startPairing();
    }
  }

}
