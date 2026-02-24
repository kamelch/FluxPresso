#pragma once
#include "../common_types.h"
#include "../logic/safety.h"

namespace FaultMgr {
    // Returns true if this fault may be auto-cleared when conditions are healthy.
    bool is_recoverable(FaultCode fc);

    // Returns true if user ACK is allowed to clear the current fault.
    // Note: for safety faults we still require conditions to be healthy.
    bool can_clear_on_ack(FaultCode fc, const SafetyInputs& in, const SafetyConfig& cfg);

    // Map ESP reset reasons to a boot fault policy.
    // Returns true if we should enter FAULT_BOOT_RESET on boot and require user ACK.
    bool reset_reason_requires_ack(uint32_t esp_reset_reason);
}
