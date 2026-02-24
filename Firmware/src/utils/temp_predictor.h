#pragma once
#include "compat.h"
#include <math.h>

/**
 * @brief Temperature Predictor for Predictive Control
 * 
 * Uses a simple thermal model to predict temperature trajectory.
 * Helps optimize preheat timing and detect anomalies.
 */
class TempPredictor {
public:
    struct ThermalModel {
        float time_constant_s = 60.0f;     // Thermal time constant (first order)
        float heat_capacity = 1.0f;         // Relative heat capacity
        float ambient_temp_c = 20.0f;       // Ambient temperature
        float max_heating_rate = 2.0f;      // Max °C/s heating rate
        float max_cooling_rate = 0.5f;      // Max °C/s cooling rate
    };

    TempPredictor()
        : model_()
        , last_temp_(0.0f)
        , last_time_ms_(0)
        , is_initialized_(false)
    {}

    TempPredictor(const ThermalModel& model)
        : model_(model)
        , last_temp_(0.0f)
        , last_time_ms_(0)
        , is_initialized_(false)
    {}

    /**
     * @brief Update predictor with new temperature measurement
     */
    void update(float temp_c, uint32_t now_ms) {
        if (!is_initialized_) {
            last_temp_ = temp_c;
            last_time_ms_ = now_ms;
            is_initialized_ = true;
            return;
        }

        last_temp_ = temp_c;
        last_time_ms_ = now_ms;
    }

    /**
     * @brief Predict temperature at a future time
     * @param lookahead_ms Time into the future (milliseconds)
     * @param heater_power Heater power 0-100%
     * @return Predicted temperature
     */
    float predict(uint32_t lookahead_ms, float heater_power) const {
        if (!is_initialized_) {
            return last_temp_;
        }

        const float dt_s = lookahead_ms / 1000.0f;
        
        // Estimate heating/cooling rate based on heater power
        float heating_rate = 0.0f;
        if (heater_power > 0) {
            heating_rate = model_.max_heating_rate * (heater_power / 100.0f);
        } else {
            // Natural cooling toward ambient
            const float temp_diff = last_temp_ - model_.ambient_temp_c;
            heating_rate = -model_.max_cooling_rate * (temp_diff / 100.0f);
        }

        // First-order exponential response
        const float alpha = exp(-dt_s / model_.time_constant_s);
        const float predicted_temp = last_temp_ + heating_rate * dt_s * (1.0f - alpha);

        return predicted_temp;
    }

    /**
     * @brief Estimate time to reach target temperature
     * @param target_temp Target temperature
     * @param heater_power Assumed heater power
     * @return Estimated time in milliseconds
     */
    uint32_t estimateTimeToTarget(float target_temp, float heater_power) const {
        if (!is_initialized_ || heater_power <= 0) {
            return 0;
        }

        const float temp_diff = target_temp - last_temp_;
        if (temp_diff <= 0) {
            return 0; // Already at or above target
        }

        const float heating_rate = model_.max_heating_rate * (heater_power / 100.0f);
        const float time_s = temp_diff / heating_rate;

        return (uint32_t)(time_s * 1000.0f);
    }


    /**
     * @brief Whether predictor has been initialized with at least one sample
     */
    bool isReady() const { return is_initialized_; }

    /**
     * @brief Update thermal model parameters
     */
    void setModel(const ThermalModel& model) {
        model_ = model;
    }

    /**
     * @brief Reset predictor state
     */
    void reset() {
        is_initialized_ = false;
        last_temp_ = 0.0f;
        last_time_ms_ = 0;
    }

private:
    ThermalModel model_;
    float last_temp_;
    uint32_t last_time_ms_;
    bool is_initialized_;
};
