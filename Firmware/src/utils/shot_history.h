#pragma once
#include "compat.h"
#ifndef ARDUINO
  #include "native_arduino_stubs.h"
#else
  #include <Preferences.h>
#endif
#include "common_types.h"

/**
 * @brief Shot History Manager with NVS Persistence
 * 
 * Stores shot history in ESP32 NVS (non-volatile storage) for analysis
 * and performance tracking. Survives power cycles and firmware updates.
 * 
 * Features:
 * - Circular buffer (oldest shots auto-deleted)
 * - NVS persistence (survives reboots)
 * - Efficient binary storage
 * - Statistics calculation
 */
class ShotHistoryManager {
public:
    ShotHistoryManager(uint8_t max_shots = 20)
        : max_shots_(max_shots)
        , shot_count_(0)
        , next_index_(0)
    {}

    /**
     * @brief Initialize shot history from NVS
     */
    bool begin() {
        if (!prefs_.begin("shot_history", false)) {
            Serial.println("Failed to open shot history namespace");
            return false;
        }
        
        // Load shot count and next index
        shot_count_ = prefs_.getUChar("count", 0);
        next_index_ = prefs_.getUChar("next_idx", 0);
        
        // Validate
        if (shot_count_ > max_shots_) {
            shot_count_ = 0;
            next_index_ = 0;
        }

        // Structure-size guard: if firmware updated and ShotHistory size changed,
        // reset history to avoid corrupted reads.
        if (shot_count_ > 0) {
            char key[16];
            snprintf(key, sizeof(key), "shot_%d", 0);
            const size_t len = prefs_.getBytesLength(key);
            if (len != 0 && len != sizeof(ShotHistory)) {
                Serial.println("Shot history format changed -> clearing history");
                clear();
            }
        }
        
        Serial.printf("Loaded %d shots from history\n", shot_count_);
        return true;
    }

    /**
     * @brief Save new shot to history
     */
    bool saveShot(const ShotHistory& shot) {
        char key[16];
        snprintf(key, sizeof(key), "shot_%d", next_index_);
        
        // Store as binary blob
        size_t written = prefs_.putBytes(key, &shot, sizeof(ShotHistory));
        if (written != sizeof(ShotHistory)) {
            Serial.println("Failed to save shot");
            return false;
        }
        
        // Update counters
        next_index_ = (next_index_ + 1) % max_shots_;
        if (shot_count_ < max_shots_) {
            shot_count_++;
        }
        
        prefs_.putUChar("count", shot_count_);
        prefs_.putUChar("next_idx", next_index_);
        
        Serial.printf("Saved shot #%d (total: %d)\n", next_index_, shot_count_);
        return true;
    }

    /**
     * @brief Load shot from history
     * @param index Shot index (0 = oldest, count-1 = newest)
     */
    bool loadShot(uint8_t index, ShotHistory& shot) {
        if (index >= shot_count_) {
            return false;
        }
        
        // Calculate actual storage index (circular buffer)
        uint8_t storage_idx;
        if (shot_count_ < max_shots_) {
            storage_idx = index;
        } else {
            storage_idx = (next_index_ + index) % max_shots_;
        }
        
        char key[16];
        snprintf(key, sizeof(key), "shot_%d", storage_idx);
        
        size_t read = prefs_.getBytes(key, &shot, sizeof(ShotHistory));
        return (read == sizeof(ShotHistory));
    }

    /**
     * @brief Get number of stored shots
     */
    uint8_t getCount() const {
        return shot_count_;
    }

    /**
     * @brief Clear all shot history
     */
    void clear() {
        prefs_.clear();
        shot_count_ = 0;
        next_index_ = 0;
        prefs_.putUChar("count", 0);
        prefs_.putUChar("next_idx", 0);
        Serial.println("Shot history cleared");
    }

    /**
     * @brief Calculate statistics from shot history
     */
    struct Statistics {
        float avg_yield_g = 0.0f;
        float avg_time_s = 0.0f;
        float avg_temp_c = 0.0f;
        float avg_stability_score = 0.0f;
        float min_yield_g = 999.0f;
        float max_yield_g = 0.0f;
        uint8_t shot_count = 0;
    };

    Statistics calculateStats() {
        Statistics stats;
        
        if (shot_count_ == 0) {
            return stats;
        }
        
        stats.shot_count = shot_count_;
        float sum_yield = 0.0f;
        float sum_time = 0.0f;
        float sum_temp = 0.0f;
        float sum_stability = 0.0f;
        
        for (uint8_t i = 0; i < shot_count_; ++i) {
            ShotHistory shot;
            if (loadShot(i, shot)) {
                sum_yield += shot.yield_g;
                sum_time += shot.total_time_ms / 1000.0f;
                sum_temp += shot.avg_temp_c;
                sum_stability += shot.temp_stability_score;
                
                stats.min_yield_g = min(stats.min_yield_g, shot.yield_g);
                stats.max_yield_g = max(stats.max_yield_g, shot.yield_g);
            }
        }
        
        stats.avg_yield_g = sum_yield / shot_count_;
        stats.avg_time_s = sum_time / shot_count_;
        stats.avg_temp_c = sum_temp / shot_count_;
        stats.avg_stability_score = sum_stability / shot_count_;
        
        return stats;
    }

    /**
     * @brief Get most recent shot
     */
    bool getLatestShot(ShotHistory& shot) {
        if (shot_count_ == 0) {
            return false;
        }
        return loadShot(shot_count_ - 1, shot);
    }

private:
    Preferences prefs_;
    uint8_t max_shots_;
    uint8_t shot_count_;
    uint8_t next_index_;
};
