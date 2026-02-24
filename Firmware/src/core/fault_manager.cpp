#include "fault_manager.h"
#include <math.h>

namespace {
    static inline bool temp_ok(const SafetyInputs& in, const SafetyConfig& cfg) {
        const bool nan = (in.temp_c != in.temp_c);
        if (nan) return false;
        if (in.temp_c > cfg.tc_fault_threshold_c) return false;
        return in.temp_c < cfg.max_temp_c;
    }
}

namespace FaultMgr {
    bool is_recoverable(FaultCode fc) {
        return fault_class(fc) == FAULTCLASS_RECOVERABLE;
    }

    bool can_clear_on_ack(FaultCode fc, const SafetyInputs& in, const SafetyConfig& cfg) {
        // LATCHED faults still require an ACK, but may additionally require healthy inputs.
        switch (fc) {
            case FAULT_TC_DISCONNECTED:
                // Latched: require thermocouple reading to return sane.
                return (in.temp_c == in.temp_c) && (in.temp_c <= cfg.tc_fault_threshold_c);
            case FAULT_OVERTEMP:
                // Latched: require below max temp and TC sane.
                return temp_ok(in, cfg);
            case FAULT_ZC_MISSING:
                // Recoverable: require stable ZC.
                return in.zc_ok;
            case FAULT_SENSOR_TIMEOUT:
                // Recoverable: if weight is optional, allow clear anyway; otherwise require it.
                if (!cfg.require_weight) return true;
                return in.weight_g > -900.0f;
            case FAULT_HEATER_STUCK_ON:
                // Latched: allow clear only once temperature is stable and below max.
                return temp_ok(in, cfg);
            case FAULT_BOOT_RESET:
                // Always allow clearing after ACK (user confirms). Outputs stay safe until then.
                return true;
            default:
                // Conservative: require inputs to be healthy.
                return (safety_check(in, cfg) == FAULT_NONE);
        }
    }

    bool reset_reason_requires_ack(uint32_t rr) {
        // Values correspond to esp_reset_reason_t.
        // Require a manual ACK for anything that suggests instability.
        // (POWERON and EXT are normal.)
        // 1: POWERON, 2: EXT, 3: SW, 4: PANIC, 5: INT_WDT, 6: TASK_WDT, 7: WDT,
        // 8: DEEPSLEEP, 9: BROWNOUT, 10: SDIO
        switch (rr) {
            case 1: // ESP_RST_POWERON
            case 2: // ESP_RST_EXT
            case 3: // ESP_RST_SW — normal after USB upload, don't fault
            case 8: // ESP_RST_DEEPSLEEP
                return false;
            default:
                return true;
        }
    }
}
