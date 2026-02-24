#pragma once
#include "../utils/compat.h"

#include <math.h>

/**
 * @brief PID Controller (production-grade)
 *
 * Features
 * - Derivative-on-measurement (avoids setpoint kick)
 * - Optional derivative low-pass filtering
 * - Optional setpoint ramping (slew-rate limit)
 * - Anti-windup that prevents integral runaway under saturation
 * - Exposes last P/I/D/output terms for diagnostics/UI
 *
 * Design choices
 * - We keep output in the same units as the actuator command (e.g., 0..100 duty).
 * - Derivative filter uses: d_filt = a*d + (1-a)*d_prev, where a∈[0..1].
 *   Smaller a => more smoothing.
 */
class PIDController {
public:
    struct Config {
        float kp = 1.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        // 0..1, smaller = stronger filtering. Typical: 0.1..0.25
        float derivative_filter = 0.15f;

        float output_min = 0.0f;
        float output_max = 100.0f;

        // Max setpoint change in units/s (0 = instant)
        float setpoint_ramp_rate = 0.0f;

        bool enable_anti_windup = true;
    };

    PIDController() : PIDController(Config{}) {}

    explicit PIDController(const Config& cfg)
        : cfg_(cfg) {}

    float update(float process_value, uint32_t now_ms) {
        // Handle first call / initialization
        if (!initialized_) {
            prev_time_ms_ = now_ms;
            prev_pv_ = process_value;

            // Snap setpoint immediately on first call
            sp_ = sp_target_;

            // Compute a reasonable first output (helps bumpless start)
            const float err = sp_ - process_value;
            last_p_ = cfg_.kp * err;
            last_i_ = integral_;
            last_d_ = 0.0f;
            last_out_ = clamp_(last_p_ + last_i_ + last_d_);

            initialized_ = true;
            return last_out_;
        }

        const uint32_t dt_ms = now_ms - prev_time_ms_;
        if (dt_ms == 0) {
            return last_out_;
        }
        prev_time_ms_ = now_ms;

        // Convert to seconds; protect against absurdly large dt (e.g., time reset)
        float dt_s = dt_ms / 1000.0f;
        if (!(dt_s > 0.0f && dt_s < 5.0f)) {
            // Reset derivative memory; keep integral to avoid a big step
            prev_pv_ = process_value;
            d_filt_ = 0.0f;
            dt_s = constrain(dt_s, 0.001f, 1.0f);
        }

        // Setpoint ramping
        if (cfg_.setpoint_ramp_rate > 0.0f) {
            const float max_step = cfg_.setpoint_ramp_rate * dt_s;
            const float delta = sp_target_ - sp_;
            if (fabsf(delta) <= max_step) {
                sp_ = sp_target_;
            } else {
                sp_ += (delta > 0.0f) ? max_step : -max_step;
            }
        } else {
            sp_ = sp_target_;
        }

        const float err = sp_ - process_value;

        // P term
        const float p_term = cfg_.kp * err;

        // D term (derivative on measurement)
        float d_term = 0.0f;
        if (cfg_.kd > 0.0f) {
            const float d_meas = -(process_value - prev_pv_) / dt_s;
            prev_pv_ = process_value;

            const float a = constrain(cfg_.derivative_filter, 0.0f, 1.0f);
            d_filt_ = a * d_meas + (1.0f - a) * d_filt_;
            d_term = cfg_.kd * d_filt_;
        } else {
            prev_pv_ = process_value;
            d_filt_ = 0.0f;
        }

        // I term (candidate)
        float i_candidate = integral_;
        if (cfg_.ki != 0.0f) {
            i_candidate = integral_ + cfg_.ki * err * dt_s;
            // Clamp integrator to a sane range (based on output range)
            const float i_lim = fabsf(cfg_.output_max - cfg_.output_min);
            if (i_lim > 0.0f) {
                i_candidate = constrain(i_candidate, -i_lim, i_lim);
            }
        }

        // Combine
        const float u_unclamped = p_term + i_candidate + d_term;
        const float u = clamp_(u_unclamped);

        // Anti-windup: only accept the new integral if it doesn't push further into saturation
        if (cfg_.ki != 0.0f) {
            if (!cfg_.enable_anti_windup) {
                integral_ = i_candidate;
            } else {
                const bool saturated_high = (u >= cfg_.output_max - 1e-6f);
                const bool saturated_low  = (u <= cfg_.output_min + 1e-6f);

                // If saturated and error would drive it further into saturation, freeze I
                const bool drives_high = (err > 0.0f);
                const bool drives_low  = (err < 0.0f);

                const bool block = (saturated_high && drives_high) || (saturated_low && drives_low);
                if (!block) {
                    integral_ = i_candidate;
                }
            }
        }

        // Publish diagnostics
        last_p_ = p_term;
        last_i_ = integral_;
        last_d_ = d_term;
        last_out_ = u;

        return last_out_;
    }

    void setSetpoint(float setpoint) { sp_target_ = setpoint; }

    float getSetpoint() const { return sp_; }

    void reset() {
        integral_ = 0.0f;
        prev_pv_ = 0.0f;
        d_filt_ = 0.0f;
        last_p_ = 0.0f;
        last_i_ = 0.0f;
        last_d_ = 0.0f;
        last_out_ = cfg_.output_min;
        prev_time_ms_ = 0;
        initialized_ = false;
    }

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& getConfig() const { return cfg_; }

    float getIntegral() const { return integral_; }

    // Diagnostics
    float lastOutput() const { return last_out_; }
    float lastPTerm() const { return last_p_; }
    float lastITerm() const { return last_i_; }
    float lastDTerm() const { return last_d_; }

private:
    Config cfg_;

    float sp_ = 0.0f;
    float sp_target_ = 0.0f;

    float integral_ = 0.0f;

    float prev_pv_ = 0.0f;
    float d_filt_ = 0.0f;

    float last_p_ = 0.0f;
    float last_i_ = 0.0f;
    float last_d_ = 0.0f;
    float last_out_ = 0.0f;

    uint32_t prev_time_ms_ = 0;
    bool initialized_ = false;

    float clamp_(float u) const {
        return constrain(u, cfg_.output_min, cfg_.output_max);
    }
};
