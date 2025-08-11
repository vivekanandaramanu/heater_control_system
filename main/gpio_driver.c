/*
 * gpio_driver.c
 *
 *  Created on: 07-Aug-2025
 *      Author: vivek
 */
#include "gpio_driver.h"
#include "esp_log.h"

static const char *TAG_GPIO = "GPIO_DRV";

void gpio_driver_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << HEATER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GPIO, "gpio_config failed: %s", esp_err_to_name(err));
    }

    // Start with heater OFF
    gpio_set_level(HEATER_GPIO, 0);
}

void heater_on(void)  { gpio_set_level(HEATER_GPIO, 1); }
void heater_off(void) { gpio_set_level(HEATER_GPIO, 0); }

void heater_toggle(void)
{
    int level = gpio_get_level(HEATER_GPIO);
    gpio_set_level(HEATER_GPIO, !level);
}