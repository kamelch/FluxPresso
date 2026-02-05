#pragma once
#include "../utils/compat.h"

/**
 * @brief Professional PID Controller with Anti-Windup, Output Limiting, and Derivative Filtering
 * 
 * Features:
 * - Velocity-form PID for numerical stability
 * - Anti-windup protection (conditional integration)
 * - Derivative filtering to reduce noise amplification
 * - Setpoint ramping for smooth transitions
 * - Bumpless transfer when enabling/disabling
 * - Configurable output limits with smooth clamping
 */
class PIDController {
public:
    struct Config {
        float kp = 1.0f;              // Proportional gain
        float ki = 0.0f;              // Integral gain
        float kd = 0.0f;              // Derivative gain
        float derivative_filter = 0.1f; // Low-pass filter coefficient (0-1, lower = more filtering)
        float output_min = 0.0f;      // Minimum output value
        float output_max = 100.0f;    // Maximum output value
        float setpoint_ramp_rate = 0.0f; // Max °C/s setpoint change (0 = instant)
        bool enable_anti_windup = true; // Enable conditional integration
    };

    PIDController() 
        : config_()
        , setpoint_(0.0f)
        , setpoint_target_(0.0f)
        , integral_(0.0f)
        , prev_error_(0.0f)
        , prev_filtered_derivative_(0.0f)
        , prev_output_(0.0f)
        , prev_time_ms_(0)
        , is_initialized_(false)
    {}

    PIDController(const Config& config)
        : config_(config)
        , setpoint_(0.0f)
        , setpoint_target_(0.0f)
        , integral_(0.0f)
        , prev_error_(0.0f)
        , prev_filtered_derivative_(0.0f)
        , prev_output_(0.0f)
        , prev_time_ms_(0)
        , is_initialized_(false)
    {}

    /**
     * @brief Update the controller (call at regular intervals)
     * @param process_value Current measured value
     * @param now_ms Current time in milliseconds
     * @return Control output value
     */
    float update(float process_value, uint32_t now_ms) {
        // First call initialization
        if (!is_initialized_) {
            prev_time_ms_ = now_ms;
            prev_error_ = setpoint_ - process_value;
            prev_output_ = config_.output_min;
            is_initialized_ = true;
            return prev_output_;
        }

        // Calculate time delta
        const uint32_t dt_ms = now_ms - prev_time_ms_;
        if (dt_ms == 0) return prev_output_; // Avoid division by zero
        
        const float dt_s = dt_ms / 1000.0f;
        prev_time_ms_ = now_ms;

        // Apply setpoint ramping if configured
        if (config_.setpoint_ramp_rate > 0.0f) {
            const float max_change = config_.setpoint_ramp_rate * dt_s;
            const float delta = setpoint_target_ - setpoint_;
            
            if (fabsf(delta) <= max_change) {
                setpoint_ = setpoint_target_;
            } else {
                setpoint_ += (delta > 0 ? max_change : -max_change);
            }
        } else {
            setpoint_ = setpoint_target_;
        }

        // Calculate error
        const float error = setpoint_ - process_value;

        // Proportional term
        const float p_term = config_.kp * error;

        // Integral term with anti-windup
        if (config_.ki > 0.0f) {
            // Only integrate if output is not saturated OR error is helping to unsaturate
            const bool output_saturated = (prev_output_ <= config_.output_min && error < 0) ||
                                         (prev_output_ >= config_.output_max && error > 0);
            
            if (!config_.enable_anti_windup || !output_saturated) {
                integral_ += config_.ki * error * dt_s;
                
                // Clamp integral to prevent excessive buildup
                const float integral_limit = config_.output_max - config_.output_min;
                integral_ = constrain(integral_, -integral_limit, integral_limit);
            }
        }

        // Derivative term with filtering (derivative on measurement to avoid setpoint kick)
        float d_term = 0.0f;
        if (config_.kd > 0.0f) {
            const float derivative = -(process_value - (setpoint_ - prev_error_)) / dt_s;
            
            // Apply low-pass filter to derivative
            const float filtered_derivative = config_.derivative_filter * derivative +
                                             (1.0f - config_.derivative_filter) * prev_filtered_derivative_;
            prev_filtered_derivative_ = filtered_derivative;
            
            d_term = config_.kd * filtered_derivative;
        }

        // Calculate output
        float output = p_term + integral_ + d_term;
        
        // Clamp output
        output = constrain(output, config_.output_min, config_.output_max);

        // Store for next iteration
        prev_error_ = error;
        prev_output_ = output;

        return output;
    }

    /**
     * @brief Set the target setpoint
     */
    void setSetpoint(float setpoint) {
        setpoint_target_ = setpoint;
    }

    /**
     * @brief Get the current (possibly ramped) setpoint
     */
    float getSetpoint() const {
        return setpoint_;
    }

    /**
     * @brief Reset the controller state (for mode changes)
     */
    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        prev_filtered_derivative_ = 0.0f;
        prev_output_ = config_.output_min;
        is_initialized_ = false;
    }

    /**
     * @brief Update controller configuration
     */
    void setConfig(const Config& config) {
        config_ = config;
    }

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const {
        return config_;
    }

    /**
     * @brief Get current integral term (for monitoring)
     */
    float getIntegral() const {
        return integral_;
    }

private:
    Config config_;
    float setpoint_;
    float setpoint_target_;
    float integral_;
    float prev_error_;
    float prev_filtered_derivative_;
    float prev_output_;
    uint32_t prev_time_ms_;
    bool is_initialized_;
};
