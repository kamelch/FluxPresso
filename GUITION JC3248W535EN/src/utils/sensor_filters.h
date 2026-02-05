#pragma once
#include "compat.h"

/**
 * @brief Moving Average Filter for Sensor Noise Reduction
 * 
 * Implements a simple moving average filter to smooth noisy sensor readings.
 * Configurable window size for different noise characteristics.
 */
template<typename T, size_t WindowSize = 5>
class MovingAverageFilter {
public:
    MovingAverageFilter() 
        : index_(0)
        , count_(0)
        , sum_(0)
    {
        for (size_t i = 0; i < WindowSize; ++i) {
            buffer_[i] = 0;
        }
    }

    /**
     * @brief Add new sample and get filtered output
     * @param value New sensor reading
     * @return Filtered value (moving average)
     */
    T update(T value) {
        // Subtract oldest value from sum
        sum_ -= buffer_[index_];
        
        // Add new value
        buffer_[index_] = value;
        sum_ += value;
        
        // Move to next position
        index_ = (index_ + 1) % WindowSize;
        
        // Track how many samples we have
        if (count_ < WindowSize) {
            count_++;
        }
        
        // Return average
        return sum_ / count_;
    }

    /**
     * @brief Get current filtered value without adding new sample
     */
    T getValue() const {
        return (count_ > 0) ? (sum_ / count_) : 0;
    }

    /**
     * @brief Reset filter state
     */
    void reset() {
        index_ = 0;
        count_ = 0;
        sum_ = 0;
        for (size_t i = 0; i < WindowSize; ++i) {
            buffer_[i] = 0;
        }
    }

    /**
     * @brief Check if filter has sufficient samples
     */
    bool isReady() const {
        return count_ >= WindowSize;
    }

private:
    T buffer_[WindowSize];
    size_t index_;
    size_t count_;
    T sum_;
};

/**
 * @brief Median Filter (3-point) for Spike Rejection
 * 
 * More aggressive filtering for sensors with occasional large spikes.
 * Uses median of last 3 samples.
 */
template<typename T>
class MedianFilter3 {
public:
    MedianFilter3()
        : count_(0)
    {
        for (int i = 0; i < 3; ++i) {
            buffer_[i] = 0;
        }
    }

    T update(T value) {
        // Shift buffer
        buffer_[2] = buffer_[1];
        buffer_[1] = buffer_[0];
        buffer_[0] = value;
        
        if (count_ < 3) {
            count_++;
            return value;  // Not enough samples yet
        }
        
        // Find median of 3 values
        T a = buffer_[0];
        T b = buffer_[1];
        T c = buffer_[2];
        
        // Simple median: middle value of three
        if (a > b) {
            if (b > c) return b;      // a > b > c
            else if (a > c) return c; // a > c > b
            else return a;            // c > a > b
        } else {
            if (a > c) return a;      // b > a > c
            else if (b > c) return c; // b > c > a
            else return b;            // c > b > a
        }
    }

    void reset() {
        count_ = 0;
        for (int i = 0; i < 3; ++i) {
            buffer_[i] = 0;
        }
    }

private:
    T buffer_[3];
    int count_;
};

/**
 * @brief Exponential Moving Average (EMA) Filter
 * 
 * Computationally efficient filter with adjustable smoothing.
 * Alpha parameter controls smoothing (0-1, lower = more smoothing).
 */
template<typename T>
class ExponentialFilter {
public:
    ExponentialFilter(float alpha = 0.2f)
        : alpha_(alpha)
        , value_(0)
        , initialized_(false)
    {}

    T update(T newValue) {
        if (!initialized_) {
            value_ = newValue;
            initialized_ = true;
            return value_;
        }
        
        value_ = alpha_ * newValue + (1.0f - alpha_) * value_;
        return value_;
    }

    T getValue() const {
        return value_;
    }

    void reset() {
        initialized_ = false;
        value_ = 0;
    }

    void setAlpha(float alpha) {
        alpha_ = constrain(alpha, 0.0f, 1.0f);
    }

private:
    float alpha_;
    T value_;
    bool initialized_;
};
