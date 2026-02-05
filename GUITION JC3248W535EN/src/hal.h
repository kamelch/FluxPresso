#pragma once
#include "common_types.h"
#include <lvgl.h>

// Initialize all hardware (Screen, Sensors, GPIO)
bool HAL_Init();

// --- SENSORS ---
// Blocking read of sensors (fast)
void HAL_ReadSensors(float &temp, float &weight, bool &zc_health);

// Tare the scale
void HAL_TareScale();

// --- ACTUATORS ---
// Apply safe outputs
void HAL_SetOutputs(uint8_t pump_percent, bool solenoid_open, uint8_t heater_duty);

// --- DISPLAY ---
// Setup LVGL display driver
void HAL_Display_Init();
// Flush callback for LVGL
void HAL_Display_Flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);
// Touch callback for LVGL
void HAL_Touch_Read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);

// --- INTERRUPTS (Internal Use) ---
void IRAM_ATTR HAL_ISR_ZeroCross();
void IRAM_ATTR HAL_ISR_DimmerTimer();