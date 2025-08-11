/*
 * main.c
 *
 *  Created on: 11-Aug-2025
 *      Author: vivek
 *
 *  Description:
 *  Entry point for the ESP32 Heater Control System.
 *  - Initializes GPIO for heater control
 *  - Initializes I²C and TMP117 temperature sensor
 *  - Creates FreeRTOS control task
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "gpio_driver.h"
#include "i2c_driver.h"
#include "heater_control.h"

#ifdef WOKWI_SIM
void mode_select(void);
#endif

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "System Init: Starting Heater Control");
    
	#ifdef WOKWI_SIM
	mode_select();   // user picks simulation mode first
	#endif

    // 1. Init GPIO (Heater pin)
    gpio_driver_init();
    ESP_LOGI(TAG, "GPIO initialized for Heater");

    // 2. Init I²C & temperature sensor
    if (i2c_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Temperature sensor initialization failed!");
        return; // stop here, sensor is critical
    }
    ESP_LOGI(TAG, "I²C & TMP117 sensor initialized");

    // 3. Create the control task
    if (xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control_task");
        return;
    }
    ESP_LOGI(TAG, "Heater Control Task started");
}

#ifdef WOKWI_SIM
extern void sim_set_mode(int mode); 

void mode_select(void)
{
    int choice = 0;
    printf("\n=== Heater Control Simulation ===\n");
    printf("Select simulation mode:\n");
    printf("  1 - Functional system (target reach)\n");
    printf("  2 - Functional system WITH overheat\n");
    printf("Enter 1 or 2 (auto-default to 1 in 10s): ");
    fflush(stdout);

    // Give time to open the Serial Monitor
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Try to read once per second, up to 10 seconds
    for (int i = 0; i < 10; ++i) {
        // Non-blocking-ish read using scanf (works in Wokwi console)
        if (scanf("%d", &choice) == 1) {
            if (choice == 2) {
                sim_set_mode(2);
                printf("\nMode 2 selected.\n");
                return;
            } else {
                sim_set_mode(1);
                printf("\nMode 1 selected.\n");
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Timeout → default to 1
    sim_set_mode(1);
    printf("\nNo input, defaulting to Mode 1.\n");
}
#endif