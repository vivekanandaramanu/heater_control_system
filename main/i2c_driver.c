/*
 * i2c_driver.c
 *
 *  Created on: 09-Aug-2025
 *      Author: vivek
 */
#include "i2c_driver.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // for pdMS_TO_TICKS
#include <stdint.h>


#ifdef WOKWI_SIM
// ------- Wokwi simulation (real I2C init + MPU6050 probe; selectable scripts) -------
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdio.h>

#define WOKWI_I2C_ADDR   0x68      // MPU6050 default address
#define WHO_AM_I_REG     0x75

static const char *TAG = "I2C_DRIVER_SIM";

/* ----------------------------- Mode selection ------------------------------ */
static int sim_mode = 1; // 1 = functional, 2 = functional + overheat
void sim_set_mode(int mode) {
    if (mode == 1 || mode == 2) sim_mode = mode;
    else sim_mode = 1;
}

/* ----------------------------- Functional script --------------------------- */
/* Path:
   0°C (3s) -> 25°C (3s) -> ramp +0.5/s to 48 -> oscillate 48↔52 (3 peaks @52)
   -> hold 50°C (5s) -> stay 50
*/
typedef enum {
    F_PH_FAULT0 = 0, F_PH_IDLE25, F_PH_RAMP, F_PH_STAB_OSC, F_PH_TARGET_HOLD, F_PH_DONE
} func_phase_t;

static func_phase_t f_phase = F_PH_FAULT0;
static float f_temp = 0.0f;
static int   f_tick = 0;
static int   f_dir  = +1;
static int   f_peaks52 = 0;

static float next_functional_temp(void)
{
    switch (f_phase) {
    case F_PH_FAULT0:
        f_temp = 0.0f;
        if (++f_tick >= 3) { f_phase = F_PH_IDLE25; f_tick = 0; }
        break;
    case F_PH_IDLE25:
        f_temp = 25.0f;
        if (++f_tick >= 3) { f_phase = F_PH_RAMP; f_tick = 0; }
        break;
    case F_PH_RAMP:
        if (f_temp < 48.0f) {
            f_temp += 2.0f; if (f_temp > 48.0f) f_temp = 48.0f;
        } else {
            f_phase = F_PH_STAB_OSC; f_dir = +1; f_tick = 0; f_peaks52 = 0;
        }
        break;
    case F_PH_STAB_OSC:
        f_temp += 0.5f * (float)f_dir;
        if (f_temp >= 52.0f) { f_temp = 52.0f; f_dir = -1; f_peaks52++; }
        else if (f_temp <= 48.0f) { f_temp = 48.0f; f_dir = +1; }
        if (f_peaks52 >= 3 && f_temp == 48.0f) {
            f_phase = F_PH_TARGET_HOLD; f_temp = 50.0f; f_tick = 0;
        }
        break;
    case F_PH_TARGET_HOLD:
        f_temp = 50.0f;
        if (++f_tick >= 5) f_phase = F_PH_DONE;
        break;
    case F_PH_DONE:
    default:
        f_temp = 50.0f;
        break;
    }
    return f_temp;
}

/* --------------------------- Overheat script ------------------------------- */
/* Same as functional through first stabilizing loop, then jump to 62°C (hold) */
typedef enum {
    O_PH_FAULT0 = 0, O_PH_IDLE25, O_PH_RAMP, O_PH_STAB_OSC, O_PH_OVERHEAT, O_PH_DONE
} over_phase_t;

static over_phase_t o_phase = O_PH_FAULT0;
static float o_temp = 0.0f;
static int   o_tick = 0;
static int   o_dir  = +1;
static int   o_peaks52 = 0;

static float next_overheat_temp(void)
{
    switch (o_phase) {
    case O_PH_FAULT0:
        o_temp = 0.0f;
        if (++o_tick >= 3) { o_phase = O_PH_IDLE25; o_tick = 0; }
        break;
    case O_PH_IDLE25:
        o_temp = 25.0f;
        if (++o_tick >= 3) { o_phase = O_PH_RAMP; o_tick = 0; }
        break;
    case O_PH_RAMP:
        if (o_temp < 48.0f) {
            o_temp += 2.0f; if (o_temp > 48.0f) o_temp = 48.0f;
        } else {
            o_phase = O_PH_STAB_OSC; o_dir = +1; o_tick = 0; o_peaks52 = 0;
        }
        break;
	case O_PH_STAB_OSC:
	    o_temp += 0.5f * (float)o_dir;
	
	    if (o_temp >= 52.0f) {
	        o_temp = 52.0f;
	        o_dir = -1;
	        o_peaks52++;  // count an upper peak at 52
	        // >>> trigger overheat on SECOND upper cycle <<<
	        if (o_peaks52 == 2) {
	            o_phase = O_PH_OVERHEAT;
	            o_tick = 0;
	            o_temp = 62.0f;   // immediate jump to overheat temp
	            break;
	        }
    } else if (o_temp <= 48.0f) {
        o_temp = 48.0f;
        o_dir = +1;
      }
      break;
    case O_PH_OVERHEAT:
        o_temp = 62.0f; // hold hot
        if (++o_tick >= 3) { o_phase = O_PH_DONE; }
        break;
    case O_PH_DONE:
    default:
        o_temp = 62.0f;
        break;
    }
    return o_temp;
}

/* ------------------------------ I2C plumbing ------------------------------- */
esp_err_t i2c_driver_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN_IO,    // GPIO21
        .scl_io_num = I2C_SCL_PIN_IO,    // GPIO22
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(err)); return err; }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(err)); return err; }
    return ESP_OK;
}

esp_err_t i2c_probe(uint8_t addr)
{
    uint8_t reg = WHO_AM_I_REG;
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, addr, &reg, 1,
                                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_probe 0x%02X: %s", addr, esp_err_to_name(err)); return err; }
    return ESP_OK;
}

esp_err_t temp_sensor_config(uint16_t val) { return ESP_OK; }

esp_err_t i2c_sensor_init(void)
{
    esp_err_t err = i2c_driver_init();
    if (err != ESP_OK) return err;

    err = i2c_probe(WOKWI_I2C_ADDR);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "WOKWI_SIM: MPU6050 present at 0x%02X; sim_mode=%d", WOKWI_I2C_ADDR, sim_mode);
    return ESP_OK;
}

esp_err_t temp_sensor_read(float *out_celsius)
{
    if (!out_celsius) return ESP_ERR_INVALID_ARG;
    *out_celsius = (sim_mode == 2) ? next_overheat_temp() : next_functional_temp();
    return ESP_OK;
}

#else
// ------- Real I2C implementation (TMP117) -------

static const char *TAG = "I2C_DRIVER";

esp_err_t i2c_driver_init(void)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN_IO,
        .scl_io_num = I2C_SCL_PIN_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
#if ESP_IDF_VERSION_MAJOR >= 4
        .clk_flags = 0, // default
#endif
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode,
                             I2C_MASTER_RX_BUFF_DISABLE, I2C_MASTER_TX_BUFF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t i2c_sensor_init(void)
{
    esp_err_t err = i2c_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_init failed");
        return err;
    }

    err = i2c_probe(TMP117_I2C_ADDR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TMP117 not found at 0x%02X", TMP117_I2C_ADDR);
        return err;
    }

    err = temp_sensor_config(TMP117_CONFIG_DEFAULT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TMP117 configuration failed");
        return err;
    }

    ESP_LOGI(TAG, "i2c_sensor_init successful");
    return ESP_OK;
}

esp_err_t i2c_probe(uint8_t addr)
{
    uint8_t reg = TMP117_REG_TEMP; // probe by pointing to TEMP reg
    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM, addr,
        &reg, 1,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_probe at 0x%02X failed: %s", addr, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "i2c_probe at 0x%02X is OK", addr);
    return ESP_OK;
}

esp_err_t temp_sensor_config(uint16_t config_value)
{
    uint8_t data[3];
    data[0] = TMP117_REG_CONFIG;
    data[1] = (uint8_t)((config_value >> 8) & 0xFF); // MSB
    data[2] = (uint8_t)(config_value & 0xFF);        // LSB

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM, TMP117_I2C_ADDR,
        data, sizeof(data),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write config 0x%04X failed: %s", config_value, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "TMP117 config write OK (0x%04X)", config_value);
    return ESP_OK;
}

esp_err_t temp_sensor_read(float *out_celsius)
{
    if (!out_celsius) return ESP_ERR_INVALID_ARG;

    uint8_t reg = TMP117_REG_TEMP;
    uint8_t data[2] = {0, 0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM, TMP117_I2C_ADDR,
        &reg, 1,
        data, 2,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write_read_device failed: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t raw = (uint16_t)((data[0] << 8) | data[1]); // MSB first
    *out_celsius = raw * TMP117_LSB_C;
    return ESP_OK;
}

#endif

