#pragma once
#include <stdint.h>
#include "../utils/compat.h"

// Simple, deterministic pump shaping:
// - ramp up at the start of FLOW
// - optional decline near the end (time or weight cutoff)
//
// This is intentionally small and dependency-free so it can be reused in tests.

struct FlowShaperConfig {
    bool enable = true;
    uint16_t ramp_up_ms = 3000;
    uint16_t ramp_down_ms = 3000;
    float end_fraction = 0.55f; // fraction of base at the end
};

static inline uint8_t flow_shaper_apply(uint8_t base_percent,
                                        uint32_t elapsed_ms,
                                        uint32_t remaining_ms,
                                        const FlowShaperConfig& c) {
    if (!c.enable) return base_percent;
    float scale = 1.0f;

    // Ramp-up
    if (c.ramp_up_ms > 0 && elapsed_ms < c.ramp_up_ms) {
        scale *= (float)elapsed_ms / (float)c.ramp_up_ms;
        if (scale < 0.08f) scale = 0.08f; // avoid "no flow" at start
    }

    // Ramp-down / decline
    if (c.ramp_down_ms > 0 && remaining_ms < c.ramp_down_ms) {
        const float t = 1.0f - ((float)remaining_ms / (float)c.ramp_down_ms); // 0..1
        const float endf = constrain(c.end_fraction, 0.1f, 1.0f);
        const float decline = 1.0f - t * (1.0f - endf);
        scale *= decline;
    }

    uint16_t v = (uint16_t)((float)base_percent * scale);
    if (v > 100) v = 100;
    return (uint8_t)v;
}
