/*
 * heater_control.c
 *
 *  Created on: 11-Aug-2025
 *      Author: vivek
 */

/**
 * @file heater_control.c
 * @brief Heater control state machine (ESP-IDF, TMP117 over I²C).
 */

#include "heater_control.h"
#include "i2c_driver.h"
#include "gpio_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "HEATER_CTRL";

/* ------------------------- Internal state & constants ---------------------- */

static ctrl_state_t state = S_IDLE_STATE;
static float  prev_c = 0.0f;
static bool   heater_on_flag = false;
static uint32_t stable_count = 0;
static uint32_t target_hold_ms = 5000; // 5 s
static uint32_t hold_accum = 0;

static uint8_t fault_recover_cnt = 0;
#define FAULT_RECOVER_SAMPLES  3

// Forward declarations for helpers placed after control_task()
static inline const char* ui_state_label(ctrl_state_t s);
static inline void restore_state_from_temp(float tC);

/* ------------------------------ Public API -------------------------------- */

void control_task(void *arg)
{
    const float IDLE_MIN  = 18.0f, IDLE_MAX  = 32.0f;
    const float STAB_LOW  = 48.0f, STAB_HIGH = 52.0f;
    const float TARGET    = 50.0f;
    const float EPSILON   = 0.05f;
    const float OVERHEAT  = 60.0f;

    for (;;) {
        float tC = 0.0f;

        // 1) Read sensor
        if (temp_sensor_read(&tC) != ESP_OK) {
            state = S_FAULT;
            heater_set(false);
            ESP_LOGE(TAG, "Sensor fault");
            ESP_LOGI(TAG, "STATE=%s, T=NaN, heater=%s",
                     ui_state_label(state), heater_on_flag ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 2) Guard + auto-recover
        if (guard_working_range(tC)) {
            ESP_LOGI(TAG, "STATE=%s, T=%.2f °C, heater=%s",
                     ui_state_label(state), tC, heater_on_flag ? "ON" : "OFF");
            prev_c = tC;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 3) State machine
        switch (state) {
        case S_IDLE_STATE: {
            if (tC >= IDLE_MIN && tC <= IDLE_MAX) {
                stable_count++;
            } else if (tC > IDLE_MAX) {
                heater_set(false);
                state = S_FAULT;
                ESP_LOGE(TAG, "Normal temp range not found to start heater (%.2f °C)", tC);
                break;
            } else {
                stable_count = 0;
            }

            if (stable_count >= 3) {
                heater_set(true);
                state = S_HEATING_STATE;
                ESP_LOGI(TAG, "Stable ambient; heater ON → HEATING_STATE");
            }
            break;
        }

        case S_HEATING_STATE: {
            if (tC >= OVERHEAT) {
                heater_set(false);
                state = S_OVERHEAT;
                ESP_LOGW(TAG, "Overheat! Heater OFF");
                break;
            }

            if ((tC >= IDLE_MIN && tC <= IDLE_MAX) && (tC <= prev_c)) {
                heater_set(false);
                state = S_IDLE_STATE;
                stable_count = 0;
                ESP_LOGI(TAG, "Temp fell to idle; returning to IDLE_STATE");
                break;
            }

            if (tC >= STAB_LOW) {
                if (!heater_on_flag) {
                    heater_set(true);
                    ESP_LOGI(TAG, "Heater was OFF, turning ON before stabilizing");
                }
                state = S_STABILIZING;
                ESP_LOGI(TAG, "Temp ≥ 48 °C → STABILIZING");
            } else {
                if (tC <= prev_c) {
                    ESP_LOGW(TAG, "Temp not rising but still in heating state as it is warm");
                } else {
                    //ESP_LOGI(TAG, "Heating... %.2f °C", tC);
                }
                heater_set(true);
            }
            break;
        }

        case S_STABILIZING: {
            if (tC >= OVERHEAT) {
                heater_set(false);
                state = S_OVERHEAT;
                ESP_LOGW(TAG, "Overheat! Heater OFF");
                break;
            }

            if (tC < STAB_LOW-2) {          //Fall back to heating state if below 46.0°C
                heater_set(true);
                state = S_HEATING_STATE;
                ESP_LOGI(TAG, "Dropped <48 °C → back to HEATING_STATE");
                break;
            }

          	if (tC <= STAB_LOW && tC > STAB_LOW-2) {        // ON at 48.0°C or below till 46.0°C for stabilizing
    			heater_set(true);
    			ESP_LOGI(TAG, "≤48 °C → heater ON (For Stabilizing )");
			} else if (tC >= STAB_HIGH) {                   // OFF at 52.0°C or above for stabilizing
			    heater_set(false);
			    ESP_LOGI(TAG, "≥52 °C → heater OFF (For Stabilizing)");
			}

            if (fabsf(tC - TARGET) <= EPSILON) {
                if (hold_accum < target_hold_ms) hold_accum += 1000;
            } else {
                hold_accum = 0;
            }
            if (hold_accum >= target_hold_ms) {
                heater_set(false);
                state = S_TARGET_REACHED;
                ESP_LOGI(TAG, "Target reached (50 °C held 5 s) → OFF");
            }
            break;
        }

        case S_TARGET_REACHED:
            heater_set(false);
            break;

        case S_OVERHEAT:
            heater_set(false);
            break;

        case S_FAULT:
        default:
            heater_set(false);
            break;
        }

        // 4) Status line
        ESP_LOGI(TAG, "STATE=%s, T=%.2f °C, heater=%s",
                 ui_state_label(state), tC, heater_on_flag ? "ON" : "OFF");

        prev_c = tC;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ------------------------------ Public helpers ---------------------------- */

void heater_set(bool on)
{
    if (on) {
        heater_on();
        heater_on_flag = true;
    } else {
        heater_off();
        heater_on_flag = false;
    }
}

bool guard_working_range(float tC)
{
    const float WORK_MIN = 18.0f;
    const float WORK_MAX = 80.0f;

    if (tC < WORK_MIN || tC > WORK_MAX) {
        heater_set(false);
        state = S_FAULT;
        fault_recover_cnt = 0;
        ESP_LOGE(TAG, "Out-of-range temperature: %.2f °C → FAULT (heater OFF)", tC);
        return true;
    }

    if (state == S_FAULT) {
        if (++fault_recover_cnt >= FAULT_RECOVER_SAMPLES) {
            restore_state_from_temp(tC);
            fault_recover_cnt = 0;
        }
        return true;
    }

    fault_recover_cnt = 0;
    return false;
}

/* -------------------- Helpers placed AFTER control_task() ------------------ */

static inline const char* ui_state_label(ctrl_state_t s)
{
    switch (s) {
        case S_IDLE_STATE:     return "idle state";
        case S_HEATING_STATE:  return "heating";
        case S_STABILIZING:    return "stabilizing";
        case S_TARGET_REACHED: return "target reached";
        case S_OVERHEAT:       return "overheating";
        case S_FAULT:          return "fault";
        default:               return "unknown";
    }
}

static inline void restore_state_from_temp(float tC)
{
    if (tC >= 18.0f && tC <= 32.0f) {
        heater_set(false);
        state = S_IDLE_STATE;
        ESP_LOGW(TAG, "Recover → IDLE_STATE (%.2f °C)", tC);
    } else if (tC < 48.0f) {            // 32–48 °C
        heater_set(true);
        state = S_HEATING_STATE;
        ESP_LOGW(TAG, "Recover → HEATING_STATE (%.2f °C, heater ON)", tC);
    } else if (tC <= 52.0f) {           // 48–52 °C
        if (!heater_on_flag) heater_set(true);
        state = S_STABILIZING;
        ESP_LOGW(TAG, "Recover → STABILIZING (%.2f °C)", tC);
    } else if (tC < 60.0f) {            // 52–60 °C
        heater_set(false);
        state = S_HEATING_STATE;        // cooling path with heater OFF
        ESP_LOGW(TAG, "Recover → cooling via HEATING_STATE (%.2f °C)", tC);
    } else {                            // ≥60 °C
        heater_set(false);
        state = S_OVERHEAT;
        ESP_LOGW(TAG, "Recover → OVERHEAT (%.2f °C)", tC);
    }
}


