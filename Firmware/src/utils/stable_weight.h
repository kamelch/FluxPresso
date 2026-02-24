// stable_weight.h
// -------------------------------------------------------------
// "Wiggle" suppressor for HX711 weight readout.
//
// Problem:
// - Under vibration (pump, grinder, EMI), the scale value can "wiggle" between
//   two levels (e.g., 100g <-> 102g). Classic smoothing still flickers.
//
// Solution:
// - 1-second **block averaging** (integration) + **hysteresis**.
//   This is a standard industrial technique: vibrations are roughly symmetric,
//   so +peaks/-valleys cancel out over the block window.
//
// Behavior:
// - Read the scale continuously (at control-loop rate) and accumulate samples.
// - Every 1000ms, compute the block average.
// - Update the displayed value only if it shifted by >= threshold (default 0.5g),
//   otherwise keep the old value (optionally follow ultra-slow drift).
//
// IMPORTANT:
// - This is intended for *UI display* (and optionally non-time-critical logic).
// - Do NOT use the stabilized output for high-frequency flow/derivative math.
// -------------------------------------------------------------

#pragma once

#include <math.h>
#include <stdint.h>

class StableWeight {
public:
    struct Config {
        // Block update interval in milliseconds.
        uint32_t block_interval_ms = 1000;

        // Only update the displayed value if the block average shifts by >= this.
        float shift_threshold_g = 0.5f;

        // Optional drift tracking when stable (0..1).
        // Set to 0.0 to completely "lock" unless threshold is exceeded.
        // If you have slow thermal creep you want to follow, try 0.01..0.03.
        float drift_alpha = 0.0f;
    };

    StableWeight() = default;
    explicit StableWeight(const Config& cfg) : cfg_(cfg) {}

    void reset(float initial_g = 0.0f) {
        inited_ = false;
        displayed_g_ = initial_g;
        baseline_g_ = initial_g;

        last_block_ms_ = 0;
        block_sum_ = 0.0;
        block_n_ = 0;

        // median-of-3 buffer
        w1_ = w2_ = w3_ = initial_g;
    }

    // Update with a new weight sample.
    // - dynamic_mode is kept for future (e.g., different thresholds during shots).
    //   Currently: still updates only at block boundaries to suppress wiggle.
    float update(uint32_t now_ms, float weight_g, bool /*dynamic_mode*/) {
        if (!isfinite(weight_g)) return displayed_g_;

        if (!inited_) {
            // Immediate first value so UI isn't blank for 1s.
            inited_ = true;
            displayed_g_ = weight_g;
            baseline_g_ = weight_g;
            last_block_ms_ = now_ms;
            block_sum_ = 0.0;
            block_n_ = 0;
            w1_ = w2_ = w3_ = weight_g;
            return displayed_g_;
        }

        // Median-of-3 spike rejection (cheap insurance against electrical spikes).
        w1_ = w2_;
        w2_ = w3_;
        w3_ = weight_g;
        const float w_med = median3_(w1_, w2_, w3_);

        // Accumulate for the current block.
        block_sum_ += (double)w_med;
        block_n_++;

        const uint32_t interval = (cfg_.block_interval_ms < 100) ? 100 : cfg_.block_interval_ms;
        if ((now_ms - last_block_ms_) >= interval) {
            if (block_n_ > 0) {
                const float avg = (float)(block_sum_ / (double)block_n_);
                const float d = avg - baseline_g_;

                if (fabsf(d) >= cfg_.shift_threshold_g) {
                    baseline_g_ = avg;
                } else {
                    // Follow slow drift without reacting to wiggle.
                    baseline_g_ = baseline_g_ + cfg_.drift_alpha * d;
                }
                displayed_g_ = baseline_g_;
            }

            // Reset block for next interval.
            last_block_ms_ = now_ms;
            block_sum_ = 0.0;
            block_n_ = 0;
        }

        return displayed_g_;
    }

    float displayed() const { return displayed_g_; }
    float baseline() const { return baseline_g_; }
    const Config& config() const { return cfg_; }
    Config& config() { return cfg_; }

private:
    Config cfg_{};
    bool inited_ = false;

    float displayed_g_ = 0.0f;
    float baseline_g_  = 0.0f;

    // Block averaging state
    uint32_t last_block_ms_ = 0;
    double block_sum_ = 0.0;
    uint16_t block_n_ = 0;

    // Median-of-3 buffer
    float w1_ = 0.0f, w2_ = 0.0f, w3_ = 0.0f;

    static inline float median3_(float a, float b, float c) {
        if (a > b) { float t=a; a=b; b=t; }
        if (b > c) { float t=b; b=c; c=t; }
        if (a > b) { float t=a; a=b; b=t; }
        return b;
    }
};
