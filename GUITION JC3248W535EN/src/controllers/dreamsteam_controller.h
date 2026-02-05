#pragma once
#include "../utils/compat.h"

/**
 * @brief DreamSteam Controller - Professional pressure-supported steam system
 * 
 * Features:
 * - Dynamic boiler refill during steaming
 * - Steam load detection via temperature drop rate
 * - Adaptive pulse timing based on load intensity
 * - Multiple safety interlocks
 * - Overfill prevention with cumulative tracking
 * - Steam valve detection (optional microswitch support)
 * - Configurable aggressiveness levels
 */
class DreamSteamController {
public:
    enum class Mode {
        MODE_DISABLED,      // No steam assist
        MODE_CONSERVATIVE,  // Safe, sensor-less mode
        MODE_AGGRESSIVE,    // With steam valve switch
        MODE_ADAPTIVE       // Self-tuning based on response
    };

    struct Config {
        Mode mode = Mode::MODE_CONSERVATIVE;
        
        // Temperature-based load detection
        float temp_load_threshold_c = 2.0f;      // Temp below setpoint to trigger
        float temp_drop_rate_threshold = -0.8f;  // °C/s drop rate to trigger
        
        // Pump pulse parameters
        uint8_t pulse_power_min = 20;            // Minimum pump power (%)
        uint8_t pulse_power_max = 40;            // Maximum pump power (%)
        uint16_t pulse_duration_ms = 180;        // Individual pulse length
        uint16_t pulse_interval_ms = 1000;       // Minimum time between pulses
        
        // Adaptive parameters
        float adaptive_gain = 0.3f;              // How quickly to adjust pulse power
        uint16_t adaptive_window_ms = 5000;      // Window for evaluating effectiveness
        
        // Safety limits
        uint32_t max_cumulative_pump_ms_per_min = 10000; // 10s total per minute
        float min_temp_for_refill_c = 130.0f;    // Don't refill if too cold
        float max_temp_for_refill_c = 150.0f;    // Don't refill if too hot
        uint16_t max_continuous_pulses = 8;      // Prevent runaway
        
        // Optional steam valve detection
        bool use_steam_valve_switch = false;
        uint8_t steam_valve_pin = 0;
        bool steam_valve_active_low = false;
    };

    struct State {
        bool is_steaming = false;
        bool steam_load_detected = false;
        bool steam_valve_open = false;
        uint8_t current_pulse_power = 0;
        uint16_t continuous_pulse_count = 0;
        uint32_t cumulative_pump_ms = 0;
        float temp_drop_rate = 0.0f;
        float effectiveness_score = 0.0f;
    };

    DreamSteamController()
        : config_()
        , last_pulse_ms_(0)
        , pulse_start_ms_(0)
        , cumulative_window_start_ms_(0)
        , continuous_pulse_count_(0)
        , current_pulse_power_(20)
        , temp_history_idx_(0)
        , adaptive_window_start_ms_(0)
        , pre_pulse_temp_(0.0f)
    {
        // Initialize temperature history
        for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) {
            temp_history_[i] = 0.0f;
            temp_history_time_[i] = 0;
        }
    }

    DreamSteamController(const Config& config)
        : config_(config)
        , last_pulse_ms_(0)
        , pulse_start_ms_(0)
        , cumulative_window_start_ms_(0)
        , continuous_pulse_count_(0)
        , current_pulse_power_(config.pulse_power_min)
        , temp_history_idx_(0)
        , adaptive_window_start_ms_(0)
        , pre_pulse_temp_(0.0f)
    {
        // Initialize temperature history
        for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) {
            temp_history_[i] = 0.0f;
            temp_history_time_[i] = 0;
        }
        
        // Setup steam valve pin if configured
        if (config_.use_steam_valve_switch && config_.steam_valve_pin > 0) {
            pinMode(config_.steam_valve_pin, INPUT_PULLUP);
        }
    }

    /**
     * @brief Get current configuration (read-only)
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Update DreamSteam controller
     * @param temp_c Current boiler temperature
     * @param setpoint_c Steam setpoint temperature
     * @param now_ms Current time
     * @param zc_ok Zero-cross signal health
     * @return Pump power percentage (0-100)
     */
    uint8_t update(float temp_c, float setpoint_c, uint32_t now_ms, bool zc_ok) {
        // Update temperature history for derivative calculation
        updateTempHistory(temp_c, now_ms);
        
        // Calculate temperature drop rate
        const float temp_drop_rate = calculateTempDropRate(now_ms);
        
        // Reset cumulative tracking every minute
        if (now_ms - cumulative_window_start_ms_ >= 60000) {
            cumulative_pump_ms_ = 0;
            cumulative_window_start_ms_ = now_ms;
            continuous_pulse_count_ = 0;
        }
        
        // Check if disabled
        if (config_.mode == Mode::MODE_DISABLED) {
            return 0;
        }
        
        // Check steam valve state if available
        bool steam_valve_open = false;
        if (config_.use_steam_valve_switch && config_.steam_valve_pin > 0) {
            const bool pin_state = digitalRead(config_.steam_valve_pin);
            steam_valve_open = config_.steam_valve_active_low ? !pin_state : pin_state;
        }
        
        // Detect steam load
        const bool temp_load = temp_c < (setpoint_c - config_.temp_load_threshold_c);
        const bool rate_load = temp_drop_rate < config_.temp_drop_rate_threshold;
        const bool steam_load_detected = temp_load || rate_load;
        
        // Cache state for external monitoring
        state_.temp_drop_rate = temp_drop_rate;
        state_.steam_load_detected = steam_load_detected;
        state_.steam_valve_open = steam_valve_open;
        state_.current_pulse_power = current_pulse_power_;
        state_.continuous_pulse_count = continuous_pulse_count_;
        state_.cumulative_pump_ms = cumulative_pump_ms_;
        
        // Safety checks
        if (!canRefill(temp_c, zc_ok, steam_valve_open, now_ms)) {
            // If we can't refill, end current pulse if active
            if (pulse_start_ms_ > 0) {
                pulse_start_ms_ = 0;
                last_pulse_ms_ = now_ms;
            }
            return 0;
        }
        
        // Pulse state machine
        const bool in_pulse = (pulse_start_ms_ > 0);
        
        if (in_pulse) {
            // Check if pulse should end
            const uint32_t pulse_elapsed = now_ms - pulse_start_ms_;
            
            if (pulse_elapsed >= config_.pulse_duration_ms) {
                // End pulse
                const uint32_t actual_pump_time = pulse_elapsed;
                cumulative_pump_ms_ += actual_pump_time;
                pulse_start_ms_ = 0;
                last_pulse_ms_ = now_ms;
                
                // Update adaptive parameters if in adaptive mode
                if (config_.mode == Mode::MODE_ADAPTIVE) {
                    updateAdaptiveParams(temp_c, now_ms);
                }
                
                return 0;
            } else {
                // Continue pulse
                return current_pulse_power_;
            }
        } else {
            // Not in pulse - check if we should start one
            if (!steam_load_detected) {
                // No load, no pulse needed
                continuous_pulse_count_ = 0;
                return 0;
            }
            
            // Check minimum interval
            if ((now_ms - last_pulse_ms_) < config_.pulse_interval_ms) {
                return 0;
            }
            
            // Start new pulse
            pulse_start_ms_ = now_ms;
            pre_pulse_temp_ = temp_c;
            adaptive_window_start_ms_ = now_ms;
            continuous_pulse_count_++;
            
            // Adjust pulse power based on mode
            if (config_.mode == Mode::MODE_ADAPTIVE) {
                // Adaptive mode: adjust based on load intensity
                const float load_intensity = fabsf(temp_drop_rate / config_.temp_drop_rate_threshold);
                const uint8_t power_range = config_.pulse_power_max - config_.pulse_power_min;
                current_pulse_power_ = config_.pulse_power_min + 
                                      (uint8_t)(power_range * constrain(load_intensity, 0.0f, 1.0f));
            } else if (config_.mode == Mode::MODE_AGGRESSIVE) {
                // Aggressive mode: use higher power
                current_pulse_power_ = config_.pulse_power_max;
            } else {
                // Conservative mode: use lower power
                current_pulse_power_ = config_.pulse_power_min;
            }
            
            return current_pulse_power_;
        }
    }

    /**
     * @brief Get current controller state for monitoring
     */
    const State& getState() const {
        return state_;
    }

    /**
     * @brief Update configuration
     */
    void setConfig(const Config& config) {
        config_ = config;
        
        // Re-setup steam valve pin if changed
        if (config_.use_steam_valve_switch && config_.steam_valve_pin > 0) {
            pinMode(config_.steam_valve_pin, INPUT_PULLUP);
        }
    }

    /**
     * @brief Reset controller state
     */
    void reset() {
        last_pulse_ms_ = 0;
        pulse_start_ms_ = 0;
        cumulative_window_start_ms_ = millis();
        cumulative_pump_ms_ = 0;
        continuous_pulse_count_ = 0;
        current_pulse_power_ = config_.pulse_power_min;
        temp_history_idx_ = 0;
        
        for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) {
            temp_history_[i] = 0.0f;
            temp_history_time_[i] = 0;
        }
    }

private:
    static constexpr int TEMP_HISTORY_SIZE = 10;
    
    Config config_;
    State state_;
    
    // Pulse timing
    uint32_t last_pulse_ms_;
    uint32_t pulse_start_ms_;
    uint32_t cumulative_window_start_ms_;
    uint32_t cumulative_pump_ms_;
    uint16_t continuous_pulse_count_;
    uint8_t current_pulse_power_;
    
    // Temperature tracking
    float temp_history_[TEMP_HISTORY_SIZE];
    uint32_t temp_history_time_[TEMP_HISTORY_SIZE];
    int temp_history_idx_;
    
    // Adaptive learning
    uint32_t adaptive_window_start_ms_;
    float pre_pulse_temp_;

    void updateTempHistory(float temp_c, uint32_t now_ms) {
        temp_history_[temp_history_idx_] = temp_c;
        temp_history_time_[temp_history_idx_] = now_ms;
        temp_history_idx_ = (temp_history_idx_ + 1) % TEMP_HISTORY_SIZE;
    }

    float calculateTempDropRate(uint32_t now_ms) {
        // Find oldest valid entry
        int oldest_idx = temp_history_idx_;
        for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) {
            if (temp_history_time_[oldest_idx] == 0) {
                oldest_idx = (oldest_idx + 1) % TEMP_HISTORY_SIZE;
            } else {
                break;
            }
        }
        
        if (temp_history_time_[oldest_idx] == 0) {
            return 0.0f; // Not enough history
        }
        
        // Calculate rate from oldest to newest
        const int newest_idx = (temp_history_idx_ - 1 + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
        const uint32_t dt_ms = temp_history_time_[newest_idx] - temp_history_time_[oldest_idx];
        
        if (dt_ms < 500) {
            return 0.0f; // Need at least 500ms of data
        }
        
        const float dt_s = dt_ms / 1000.0f;
        const float dtemp = temp_history_[newest_idx] - temp_history_[oldest_idx];
        
        return dtemp / dt_s;
    }

    bool canRefill(float temp_c, bool zc_ok, bool steam_valve_open, uint32_t now_ms) {
        // Check zero-cross signal
        if (!zc_ok) {
            return false;
        }
        
        // Temperature range check
        if (temp_c < config_.min_temp_for_refill_c || temp_c > config_.max_temp_for_refill_c) {
            return false;
        }
        
        // Cumulative time limit
        if (cumulative_pump_ms_ >= config_.max_cumulative_pump_ms_per_min) {
            return false;
        }
        
        // Continuous pulse limit
        if (continuous_pulse_count_ >= config_.max_continuous_pulses) {
            return false;
        }
        
        // Steam valve check (if configured)
        if (config_.use_steam_valve_switch && !steam_valve_open) {
            return false;
        }
        
        return true;
    }

    void updateAdaptiveParams(float temp_c, uint32_t now_ms) {
        // Evaluate effectiveness of last pulse
        const uint32_t window_elapsed = now_ms - adaptive_window_start_ms_;
        
        if (window_elapsed < config_.adaptive_window_ms) {
            return; // Not enough time to evaluate
        }
        
        // Calculate temperature recovery
        const float temp_recovery = temp_c - pre_pulse_temp_;
        
        // Effectiveness: positive if temp recovered, negative if continued to drop
        const float effectiveness = temp_recovery / config_.adaptive_window_ms * 1000.0f; // °C/s
        state_.effectiveness_score = effectiveness;
        
        // Adjust pulse power based on effectiveness
        if (effectiveness < -0.2f) {
            // Pulse was not effective enough, increase power
            current_pulse_power_ = constrain(
                current_pulse_power_ + (uint8_t)(config_.pulse_power_max * config_.adaptive_gain),
                config_.pulse_power_min,
                config_.pulse_power_max
            );
        } else if (effectiveness > 0.5f) {
            // Pulse was very effective, can reduce power slightly
            current_pulse_power_ = constrain(
                current_pulse_power_ - (uint8_t)(config_.pulse_power_max * config_.adaptive_gain * 0.5f),
                config_.pulse_power_min,
                config_.pulse_power_max
            );
        }
    }
};
