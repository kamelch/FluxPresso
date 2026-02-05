#pragma once
#include "compat.h"

/**
 * @brief Pump Power Ramping Controller
 * 
 * Provides smooth pump power transitions to reduce mechanical stress,
 * minimize pressure spikes, and improve extraction consistency.
 * 
 * Features:
 * - Configurable ramp time (100-500ms typical)
 * - Prevents sudden pressure changes
 * - Reduces mechanical wear
 * - Improves shot consistency
 */
class PumpRampController {
public:
    struct Config {
        uint16_t ramp_up_ms = 150;      // Time to ramp from 0 to 100%
        uint16_t ramp_down_ms = 100;    // Time to ramp from 100% to 0%
        uint8_t min_power_step = 2;     // Minimum % change per update
        bool enable_ramping = true;      // Can disable for manual mode
    };

    // NOTE: Avoid default-constructing Config() in a default parameter.
    // Some toolchains (and certain C++ standard modes) can choke on this
    // when Config has in-class member initializers.
    PumpRampController() : PumpRampController(Config{}) {}

    explicit PumpRampController(const Config& config)
        : config_(config)
        , current_power_(0)
        , target_power_(0)
        , last_update_ms_(0)
    {}

    /**
     * @brief Set target pump power with ramping
     * @param target Target power 0-100%
     * @param now_ms Current time in milliseconds
     * @return Current ramped power output
     */
    uint8_t setTarget(uint8_t target, uint32_t now_ms) {
        target = constrain(target, 0, 100);
        
        // If ramping disabled, jump to target immediately
        if (!config_.enable_ramping) {
            current_power_ = target;
            target_power_ = target;
            last_update_ms_ = now_ms;
            return current_power_;
        }
        
        // Store new target
        target_power_ = target;
        
        // First call initialization
        if (last_update_ms_ == 0) {
            last_update_ms_ = now_ms;
            return current_power_;
        }
        
        // Calculate time since last update
        const uint32_t dt_ms = now_ms - last_update_ms_;
        if (dt_ms == 0) {
            return current_power_;
        }
        
        last_update_ms_ = now_ms;
        
        // Already at target
        if (current_power_ == target_power_) {
            return current_power_;
        }
        
        // Determine ramp direction and rate
        const bool ramping_up = target_power_ > current_power_;
        const uint16_t ramp_time_ms = ramping_up ? config_.ramp_up_ms : config_.ramp_down_ms;
        
        // Calculate maximum change for this time step
        // power_per_ms = 100% / ramp_time_ms
        const float power_per_ms = 100.0f / ramp_time_ms;
        const float max_change = power_per_ms * dt_ms;
        
        // Ensure minimum step size for perceptibility
        const float actual_change = max(max_change, (float)config_.min_power_step);
        
        // Apply ramping
        if (ramping_up) {
            const float new_power = current_power_ + actual_change;
            current_power_ = (uint8_t)min(new_power, (float)target_power_);
        } else {
            const float new_power = current_power_ - actual_change;
            current_power_ = (uint8_t)max(new_power, (float)target_power_);
        }
        
        return current_power_;
    }

    /**
     * @brief Get current ramped power
     */
    uint8_t getCurrentPower() const {
        return current_power_;
    }

    /**
     * @brief Check if ramping is complete
     */
    bool isAtTarget() const {
        return current_power_ == target_power_;
    }

    /**
     * @brief Force immediate power change (emergency stop)
     */
    void forceStop() {
        current_power_ = 0;
        target_power_ = 0;
    }

    /**
     * @brief Update configuration
     */
    void setConfig(const Config& config) {
        config_ = config;
    }

    /**
     * @brief Reset controller state
     */
    void reset() {
        current_power_ = 0;
        target_power_ = 0;
        last_update_ms_ = 0;
    }

private:
    Config config_;
    uint8_t current_power_;
    uint8_t target_power_;
    uint32_t last_update_ms_;
};
