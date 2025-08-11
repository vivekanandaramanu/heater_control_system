/*
 * i2c_driver.h
 *
 *  Created on: 09-Aug-2025
 *      Author: vivek
 */

#ifndef MAIN_I2C_DRIVER_H_
#define MAIN_I2C_DRIVER_H_

#include "esp_err.h"
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WOKWI_SIM
// Set simulation mode from the console (1 = functional, 2 = functional+overheat)
void sim_set_mode(int mode);
#endif

/* I2C master config (ESP32-WROOM defaults) */
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_SDA_PIN_IO        GPIO_NUM_21
#define I2C_SCL_PIN_IO        GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ    100000      // 100 kHz is plenty for TMP117
#define I2C_MASTER_TIMEOUT_MS 1000        // 1 s timeout

/* Disable master ring buffers (we do synchronous transactions) */
#define I2C_MASTER_TX_BUFF_DISABLE 0
#define I2C_MASTER_RX_BUFF_DISABLE 0

/* TMP117 address & registers */
#define TMP117_I2C_ADDR   0x48
#define TMP117_REG_TEMP   0x00
#define TMP117_REG_CONFIG 0x01

/* Example config: continuous conv, ~1Hz, default averaging (0x0220) */
#define TMP117_CONFIG_DEFAULT 0x0220

/* 1 LSB = 1/128 °C */
#define TMP117_LSB_C 0.0078125f

/** Init I2C master (pins, freq), install driver. */
esp_err_t i2c_driver_init(void);

/** Initialize sensor path: start I2C, probe TMP117, write default config. */
esp_err_t i2c_sensor_init(void);

/** Probe an I2C address by writing the TEMP register address (no read). */
esp_err_t i2c_probe(uint8_t addr);

/** Write TMP117 configuration register (MSB first). */
esp_err_t temp_sensor_config(uint16_t config_value);

/** Read temperature (°C) from TMP117. */
esp_err_t temp_sensor_read(float *out_celsius);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_I2C_DRIVER_H_ */