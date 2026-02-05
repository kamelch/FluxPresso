#pragma once
#include <stdint.h>

static constexpr float TEMP_SETPOINT_BREW   = 93.0f;
static constexpr float TEMP_SETPOINT_STEAM  = 155.0f;
static constexpr float MAX_TEMP_SAFETY      = 165.0f;
static constexpr uint32_t MAX_SHOT_TIME_S   = 60;

static constexpr uint32_t SSR_WINDOW_MS     = 1000;

static constexpr float SCALE_CALIBRATION    = -420.0f;

static constexpr uint32_t DIMMER_MIN_DELAY_US = 150;
static constexpr uint32_t DIMMER_MAX_DELAY_US = 9500;
static constexpr uint32_t DIMMER_PULSE_US     = 60;

static constexpr bool ENABLE_ZC_MONITOR = true;
static constexpr bool ENABLE_SHELLY     = true;
static constexpr bool ENABLE_HEATER_SSR = true;
