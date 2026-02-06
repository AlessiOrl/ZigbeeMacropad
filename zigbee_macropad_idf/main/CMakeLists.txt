idf_component_register(
    SRCS "esp_zb_macropad.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS ""
    REQUIRES "esp_pm"
    PRIV_REQUIRES
        "driver"
        "esp_common"
        "esp_timer"
        "espressif__esp-zigbee-lib"
        "freertos"
        "log"
        "nvs_flash"
)
