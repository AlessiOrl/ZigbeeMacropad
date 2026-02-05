
#include <stdint.h>
#include "esp_err.h"
#include "rgb_led.h"

/*
 * LED hardware has been removed from this project.
 * Keep these functions as no-op stubs.
 */

esp_err_t rgb_led_init(void)
{
    return ESP_OK;
}

void rgb_led_set_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness_percent)
{
    (void)r;
    (void)g;
    (void)b;
    (void)brightness_percent;
}

void rgb_led_off(void)
{
}

