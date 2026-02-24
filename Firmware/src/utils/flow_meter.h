#pragma once
#include "compat.h"
#include <math.h>
#include <stdint.h>

// Production-grade flow estimation from a noisy weight signal (HX711).
//
// Goals:
// - Keep the estimator stable under vibration ("wiggle") while staying responsive enough
//   for dimmer-driven vibe pump control.
// - Provide reliable flow_valid detection and a smooth ml/s signal.
//
// Strategy:
// - Median-of-3 spike rejection
// - EMA on weight
// - Windowed *regression slope* (g/s) computed over the last N samples
//   (this cancels symmetric vibration much better than a 1-step derivative)
// - EMA on flow
// - Debounced "flow valid" detection
// - EWMA variance of flow for channeling/instability detection
class FlowMeter {
public:
    struct Config {
        // Weight filtering
        float weight_ema_alpha = 0.10f;   // 0..1 (higher = less smoothing)

        // Flow filtering
        float flow_ema_alpha   = 0.22f;   // 0..1

        // Windowed slope (regression) settings
        // With CORE_TASK at 20Hz, window 10..14 gives ~0.5..0.7s effective slope window.
        uint8_t slope_window_samples = 12;
        float   slope_min_span_s     = 0.25f; // require at least this much time span

        // Validity gating
        float flow_start_th_gps = 0.35f;  // g/s
        float flow_stop_th_gps  = 0.18f;  // g/s
        uint32_t flow_start_debounce_ms = 300;
        uint32_t flow_stop_debounce_ms  = 600;

        // Sanity clamps
        float flow_min_gps = -2.0f;
        float flow_max_gps = 12.0f;

        // Density for ml/s conversion (g/ml). Espresso ~1.00..1.05
        float density_g_per_ml = 1.00f;

        // Variance EWMA (0..1). Lower => longer memory.
        float var_alpha = 0.08f;

        // Require stable dt window for bookkeeping (control tick)
        float min_dt_s = 0.02f;
        float max_dt_s = 0.50f;

        // Internal buffer (kept a bit larger than slope_window)
        uint8_t buf_capacity = 20; // max 32, see implementation
    };

    struct State {
        float weight_raw_g = NAN;
        float weight_filt_g = NAN;

        float flow_inst_gps = 0.0f;  // regression slope (g/s)
        float flow_ema_gps  = 0.0f;
        float flow_mlps     = 0.0f;

        bool  flow_valid = false;
        uint32_t flow_start_ms = 0;

        float flow_var = 0.0f;     // EWMA variance of flow_ema_gps
        float flow_std = 0.0f;     // sqrt(flow_var)

        float total_g = 0.0f;
        float avg_flow_gps = 0.0f;
    };

    FlowMeter() : cfg_() {}
    explicit FlowMeter(const Config& cfg) : cfg_(cfg) {}

    void reset(uint32_t now_ms);
    void start_shot(uint32_t now_ms, float current_weight_g);
    void update(uint32_t now_ms, float weight_g, bool shot_active);

    const State& state() const { return st_; }
    const Config& config() const { return cfg_; }
    Config& config() { return cfg_; }

private:
    Config cfg_;
    State st_;

    // median-of-3 buffer
    float w1_ = 0, w2_ = 0, w3_ = 0;

    // Regression buffer (ring)
    static constexpr uint8_t kMaxBuf = 32;
    float    wbuf_[kMaxBuf] = {0};
    uint32_t tbuf_[kMaxBuf] = {0};
    uint8_t  head_ = 0;      // next write
    uint8_t  count_ = 0;     // number of valid samples

    float tare_g_ = 0.0f;
    float prev_w_filt_ = 0.0f;
    uint32_t prev_ms_ = 0;

    uint32_t above_ms_ = 0;
    uint32_t below_ms_ = 0;

    static inline float clampf_(float x, float lo, float hi) {
        return (x < lo) ? lo : (x > hi ? hi : x);
    }
    static inline float ema_(float prev, float x, float a) {
        return prev + a * (x - prev);
    }
    static inline float median3_(float a, float b, float c) {
        if (a > b) { float t=a; a=b; b=t; }
        if (b > c) { float t=b; b=c; c=t; }
        if (a > b) { float t=a; a=b; b=t; }
        return b;
    }

    void push_sample_(uint32_t now_ms, float w_filt);
    bool compute_slope_gps_(float &out_slope_gps) const;
};
