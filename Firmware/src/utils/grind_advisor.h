#pragma once

#include "compat.h"
#ifndef ARDUINO
  #include "native_arduino_stubs.h"
#else
  #include <Arduino.h>
  #include <Preferences.h>
#endif

#include "../common_types.h"

// Statistical grind advisor (appliance-grade).
// Keeps small "bins" (grind -> EWMA errors) per profile×roast.
// Uses write-debouncing to avoid flash wear.

class GrindAdvisor {
public:
    struct Entry {
        uint16_t grind_level = 0;
        uint16_t count = 0;
        float ema_abs_time_error_s = 0.0f;
        float ema_abs_yield_error_g = 0.0f;
    };

    static constexpr uint8_t MAX_ENTRIES = 30;

    struct Model {
        uint32_t version = 1;
        uint8_t used = 0;
        Entry entries[MAX_ENTRIES]{};
    };

    bool begin() {
        // NVS namespace shared with settings
        const bool ok = prefs_.begin("gaggia", false);
        // Mark caches as not loaded
        for (uint8_t p = 0; p < MAX_PROFILES; ++p) {
            for (uint8_t r = 0; r < 3; ++r) {
                cache_[p][r].loaded = false;
                cache_[p][r].dirty = false;
                cache_[p][r].pending = 0;
                cache_[p][r].last_save_ms = 0;
                cache_[p][r].model = Model{};
            }
        }
        return ok;
    }

    void end() { prefs_.end(); }

    // Flush any pending models to NVS (call occasionally, e.g. on shot end or every minute).
    void flush() {
        const uint32_t now = millis();
        for (uint8_t p = 0; p < MAX_PROFILES; ++p) {
            for (uint8_t r = 0; r < 3; ++r) {
                Cache &c = cache_[p][r];
                if (c.loaded && c.dirty) {
                    save_model((uint8_t)p, (RoastType)r, c.model);
                    c.dirty = false;
                    c.pending = 0;
                    c.last_save_ms = now;
                }
            }
        }
    }

    // Update the model with a completed shot.
    void observe(uint8_t profile_id,
                 RoastType roast,
                 uint16_t grind_level,
                 float actual_yield_g,
                 float target_yield_g,
                 float time_s) {
        if (profile_id >= MAX_PROFILES) return;
        if (target_yield_g <= 0.1f) return;
        if (grind_level == 0) return;

        Cache &c = cache_[profile_id][(uint8_t)roast];
        ensure_loaded(profile_id, roast, c);

        // Find or allocate entry
        Model &m = c.model;
        int idx = -1;
        for (uint8_t i = 0; i < m.used; ++i) {
            if (m.entries[i].grind_level == grind_level) { idx = i; break; }
        }
        if (idx < 0) {
            if (m.used < MAX_ENTRIES) {
                idx = m.used++;
                m.entries[idx].grind_level = grind_level;
            } else {
                // Replace worst (highest error)
                float worst = -1;
                uint8_t worst_i = 0;
                for (uint8_t i = 0; i < m.used; ++i) {
                    const float score = m.entries[i].ema_abs_time_error_s + 0.5f * m.entries[i].ema_abs_yield_error_g;
                    if (score > worst) { worst = score; worst_i = i; }
                }
                idx = worst_i;
                m.entries[idx] = Entry{};
                m.entries[idx].grind_level = grind_level;
            }
        }

        Entry &e = m.entries[idx];
        const float abs_time_err = fabsf(time_s - 28.5f); // balanced target midpoint
        const float abs_yield_err = fabsf(actual_yield_g - target_yield_g);

        const float alpha = 0.25f; // EWMA
        if (e.count == 0) {
            e.ema_abs_time_error_s = abs_time_err;
            e.ema_abs_yield_error_g = abs_yield_err;
        } else {
            e.ema_abs_time_error_s = (1.0f - alpha) * e.ema_abs_time_error_s + alpha * abs_time_err;
            e.ema_abs_yield_error_g = (1.0f - alpha) * e.ema_abs_yield_error_g + alpha * abs_yield_err;
        }
        e.count = (uint16_t)min<uint32_t>(65535, (uint32_t)e.count + 1);

        // Debounced save to protect flash
        c.dirty = true;
        c.pending = (uint8_t)min<uint16_t>(255, (uint16_t)c.pending + 1);

        const uint32_t now = millis();
        const bool should_save = (c.pending >= 5) || (c.last_save_ms == 0) || ((now - c.last_save_ms) > 60000);
        if (should_save) {
            save_model(profile_id, roast, m);
            c.dirty = false;
            c.pending = 0;
            c.last_save_ms = now;
        }
    }

    // Return a suggestion and confidence 0..100.
    void suggest(uint8_t profile_id,
                 RoastType roast,
                 uint16_t current_grind,
                 uint16_t& out_grind,
                 uint8_t& out_confidence,
                 int8_t& out_delta) {
        if (profile_id >= MAX_PROFILES) {
            out_grind = current_grind;
            out_confidence = 0;
            out_delta = 0;
            return;
        }

        Cache &c = cache_[profile_id][(uint8_t)roast];
        ensure_loaded(profile_id, roast, c);

        const Model &m = c.model;
        if (m.used == 0) {
            out_grind = current_grind;
            out_confidence = 0;
            out_delta = 0;
            return;
        }

        // Choose best score
        float best_score = 1e9f;
        uint8_t best_i = 0;
        uint16_t total_count = 0;
        for (uint8_t i = 0; i < m.used; ++i) total_count = (uint16_t)min<uint32_t>(65535, total_count + m.entries[i].count);

        for (uint8_t i = 0; i < m.used; ++i) {
            const Entry &e = m.entries[i];
            if (e.count == 0) continue;
            // time dominates; yield error matters too
            const float score = e.ema_abs_time_error_s + 0.7f * e.ema_abs_yield_error_g;
            if (score < best_score) { best_score = score; best_i = i; }
        }

        const Entry &best = m.entries[best_i];
        out_grind = best.grind_level;

        // Confidence based on (a) count for this entry, and (b) share of total observations
        const float c1 = constrain((float)best.count / 12.0f, 0.0f, 1.0f);  // 12 shots ~ full confidence
        const float c2 = (total_count > 0) ? constrain((float)best.count / (float)total_count, 0.0f, 1.0f) : 0.0f;
        const float conf = 0.7f * c1 + 0.3f * c2;
        out_confidence = (uint8_t)(conf * 100.0f);

        // Delta hint: negative=finer, positive=coarser (assumes lower number = finer)
        if (current_grind == 0 || out_grind == 0) {
            out_delta = 0;
        } else {
            const int d = (int)out_grind - (int)current_grind;
            if (d < -127) out_delta = -127;
            else if (d > 127) out_delta = 127;
            else out_delta = (int8_t)d;
        }
    }

private:
    struct Cache {
        bool loaded = false;
        bool dirty = false;
        uint8_t pending = 0;
        uint32_t last_save_ms = 0;
        Model model{};
    };

    Preferences prefs_;
    Cache cache_[MAX_PROFILES][3]{};

    String key(uint8_t profile_id, RoastType roast) {
        char buf[16];
        snprintf(buf, sizeof(buf), "ga%u_%u", (unsigned)profile_id, (unsigned)roast);
        return String(buf);
    }

    void ensure_loaded(uint8_t profile_id, RoastType roast, Cache &c) {
        if (c.loaded) return;
        Model m{};
        load_model(profile_id, roast, m);
        c.model = m;
        c.loaded = true;
        c.dirty = false;
        c.pending = 0;
        c.last_save_ms = millis();
    }

    bool load_model(uint8_t profile_id, RoastType roast, Model &m) {
        const String k = key(profile_id, roast);
        const size_t len = prefs_.getBytesLength(k.c_str());
        if (len != sizeof(Model)) {
            m = Model{};
            return false;
        }
        prefs_.getBytes(k.c_str(), &m, sizeof(Model));
        if (m.version != 1 || m.used > MAX_ENTRIES) {
            m = Model{};
            return false;
        }
        return true;
    }

    void save_model(uint8_t profile_id, RoastType roast, const Model &m) {
        const String k = key(profile_id, roast);
        prefs_.putBytes(k.c_str(), &m, sizeof(Model));
    }
};
