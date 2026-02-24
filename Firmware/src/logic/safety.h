#pragma once
#include "../common_types.h"
#include "../project_config.h"
// IMPORTANT: ESP-IDF's unity headers define an `isnan` macro which can break
// `std::isnan(...)` calls during `pio test` on embedded targets.
// Use a portable NaN check that avoids that macro.
#include <math.h>

struct SafetyConfig {
    float max_temp_c = 170.0f;
    float tc_fault_threshold_c = 900.0f; // MAX31855 open circuit usually yields huge values
    bool require_weight = false;         // weight is optional for safe brewing
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

/**
 * @param idle_mode  When true, only OVERTEMP is a hard fault.
 *                   TC-disconnect and ZC-missing are logged but not faulted,
 *                   allowing the UI to remain usable while wiring is being debugged.
 *                   Set false during active brewing / steam for full safety.
 */
static inline FaultCode safety_check(const SafetyInputs& in, const SafetyConfig& cfg,
                                      bool idle_mode = false) {
    const bool temp_is_nan = (in.temp_c != in.temp_c);

    // Overtemp is always fatal (regardless of idle_mode)
    if (!temp_is_nan && in.temp_c >= cfg.max_temp_c) return FAULT_OVERTEMP;

    // In idle mode, skip TC/ZC faults so the user can use the UI and debug wiring.
    // The heater will still be driven by PID if temp is valid — if temp is NaN the PID
    // output will be 0 anyway (999°C > setpoint → no heating needed).
    if (!idle_mode) {
        if (temp_is_nan || in.temp_c > cfg.tc_fault_threshold_c) return FAULT_TC_DISCONNECTED;
        if (FAULT_ON_ZC_MISSING && ENABLE_ZC && !in.zc_ok) return FAULT_ZC_MISSING;
        if (cfg.require_weight && in.weight_g <= -900.0f) return FAULT_SENSOR_TIMEOUT;
    }

    return FAULT_NONE;
}
