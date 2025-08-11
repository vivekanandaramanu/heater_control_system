/*
 * heater_control.h
 *
 *  Created on: 11-Aug-2025
 *      Author: vivek
 */

#ifndef HEATER_CONTROL_H
#define HEATER_CONTROL_H

#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Heater control state machine states.
 */
typedef enum {
    S_IDLE_STATE = 0,     ///< Wait for ambient stability (18–32 °C)
    S_HEATING_STATE,      ///< Ramp until ≥48 °C
    S_STABILIZING,        ///< 48–52 °C band control
    S_TARGET_REACHED,     ///< 50 °C held for 5 s
    S_OVERHEAT,           ///< ≥60 °C
    S_FAULT               ///< Guard hit / sensor error
} ctrl_state_t;

/**
 * @brief FreeRTOS task: main heater control loop.
 * @param arg Unused; pass NULL.
 */
void control_task(void *arg);

/**
 * @brief Set heater output state and update internal tracking flag.
 * @param on true → ON; false → OFF.
 */
void heater_set(bool on);

/**
 * @brief Temperature guard with auto-recovery.
 * @param tC Current temperature in °C.
 * @return true if FAULT/recovery handling happened this cycle.
 */
bool guard_working_range(float tC);

#ifdef __cplusplus
}
#endif

#endif // HEATER_CONTROL_H
