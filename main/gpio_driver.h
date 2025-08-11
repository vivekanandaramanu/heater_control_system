/*
 * gpio_driver.h
 * Basic GPIO driver for heater relay control (ESP-IDF).
 *
 *  Created on: 07-Aug-2025
 *      Author: vivek
 */

#ifndef MAIN_GPIO_DRIVER_H_
#define MAIN_GPIO_DRIVER_H_

#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Heater relay GPIO (set to your board pin). */
#define HEATER_GPIO GPIO_NUM_2

/** Initialize HEATER_GPIO as output and start with heater OFF. */
void gpio_driver_init(void);

/** Turn the heater ON (drive HEATER_GPIO high). */
void heater_on(void);

/** Turn the heater OFF (drive HEATER_GPIO low). */
void heater_off(void);

/** Toggle the heater output level. */
void heater_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_GPIO_DRIVER_H_ */