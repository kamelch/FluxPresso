#pragma once
#include "../common_types.h"
// IMPORTANT: ESP-IDF's unity headers define an `isnan` macro which can break
// `std::isnan(...)` calls during `pio test` on embedded targets.
// Use a portable NaN check that avoids that macro.
#include <math.h>

struct SafetyConfig {
    float max_temp_c = 170.0f;
    float tc_fault_threshold_c = 900.0f; // MAX31855 open circuit usually yields huge values
};

struct SafetyInputs {
    float temp_c = 0.0f;
    float weight_g = 0.0f;
    bool zc_ok = true;

    // Toolchain compatibility: some GCC/Arduino combinations treat brace-init as ctor call.
    // Provide an explicit ctor so SafetyInputs{t,w,ok} always compiles.
    constexpr SafetyInputs() = default;
    constexpr SafetyInputs(float t, float w, bool z) : temp_c(t), weight_g(w), zc_ok(z) {}
};

static inline FaultCode safety_check(const SafetyInputs& in, const SafetyConfig& cfg) {
    const bool temp_is_nan = (in.temp_c != in.temp_c);
    if (temp_is_nan || in.temp_c > cfg.tc_fault_threshold_c) return FAULT_TC_DISCONNECTED;
    if (in.temp_c >= cfg.max_temp_c) return FAULT_OVERTEMP;
    if (!in.zc_ok) return FAULT_ZC_MISSING;
    if (in.weight_g <= -900.0f) return FAULT_SENSOR_TIMEOUT;
    return FAULT_NONE;
}
