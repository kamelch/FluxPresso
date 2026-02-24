#pragma once
#include "common_types.h"
#include <lvgl.h>

// Initialize all hardware (Screen, Sensors, GPIO)
bool HAL_Init();

// Force critical outputs to a safe OFF state as early as possible.
// Call this at the very beginning of setup(), before any init that may stall (display/WiFi).
void HAL_ForceOutputsSafeEarly();

// Reset reason helpers (brownout/watchdog/etc.)
uint32_t HAL_GetResetReason();

// --- SENSORS ---
// Blocking read of sensors (fast)
void HAL_ReadSensors(float &temp, float &weight, bool &zc_health);


// Scale health helpers
uint32_t HAL_ScaleAgeMs();
bool HAL_ScaleIsStale();

// Tare the scale
void HAL_TareScale();

// --- SCALE CALIBRATION ---
// HX711 scale factor (grams per ADC unit). Negative is common depending on wiring.
void HAL_SetScaleCalibration(float scale_factor);
float HAL_GetScaleCalibration();

// Optional: persist/restore HX711 offset (tare value)
void HAL_SetScaleOffset(long offset);
long HAL_GetScaleOffset();

// --- ACTUATORS ---
// Apply safe outputs
void HAL_SetOutputs(uint8_t pump_percent, bool solenoid_open, uint8_t heater_duty);

// Convenience: force everything to a safe OFF state
void HAL_AllOutputsOff();

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
