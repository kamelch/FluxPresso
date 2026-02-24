#include "flow_meter.h"

void FlowMeter::reset(uint32_t now_ms) {
    st_ = State{};
    st_.weight_raw_g = NAN;
    st_.weight_filt_g = NAN;

    prev_ms_ = now_ms;
    prev_w_filt_ = 0.0f;
    above_ms_ = below_ms_ = 0;

    w1_ = w2_ = w3_ = 0;

    tare_g_ = 0.0f;

    // reset regression ring
    head_ = 0;
    count_ = 0;
}

void FlowMeter::start_shot(uint32_t now_ms, float current_weight_g) {
    // Best-effort tare at shot start: treat current weight as zero reference.
    reset(now_ms);
    tare_g_ = current_weight_g;

    st_.weight_raw_g  = current_weight_g;
    st_.weight_filt_g = current_weight_g;
    prev_w_filt_ = current_weight_g;

    push_sample_(now_ms, current_weight_g);

    st_.flow_start_ms = 0;
    st_.flow_valid = false;
    st_.total_g = 0.0f;
}

void FlowMeter::push_sample_(uint32_t now_ms, float w_filt) {
    const uint8_t cap = (cfg_.buf_capacity <= kMaxBuf) ? cfg_.buf_capacity : kMaxBuf;
    wbuf_[head_] = w_filt;
    tbuf_[head_] = now_ms;
    head_ = (uint8_t)((head_ + 1) % cap);
    if (count_ < cap) count_++;
}

bool FlowMeter::compute_slope_gps_(float &out_slope_gps) const {
    const uint8_t cap = (cfg_.buf_capacity <= kMaxBuf) ? cfg_.buf_capacity : kMaxBuf;
    if (count_ < 4) return false;

    const uint8_t want = cfg_.slope_window_samples;
    const uint8_t n = (want > 0) ? ((want < count_) ? want : count_) : count_;
    if (n < 4) return false;

    // Oldest sample index for the last n samples
    int start = (int)head_ - (int)n;
    while (start < 0) start += cap;

    // Time span check
    const uint32_t t0 = tbuf_[start];
    const int last = (start + n - 1) % cap;
    const uint32_t tN = tbuf_[last];
    const float span_s = (tN > t0) ? ((tN - t0) / 1000.0f) : 0.0f;
    if (!(span_s >= cfg_.slope_min_span_s)) return false;

    // Linear regression slope b = cov(t,w)/var(t)
    // Use t relative to t0 to improve numerical stability.
    float sum_t = 0.0f, sum_w = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        const int idx = (start + i) % cap;
        const float ti = (tbuf_[idx] > t0) ? ((tbuf_[idx] - t0) / 1000.0f) : 0.0f;
        sum_t += ti;
        sum_w += wbuf_[idx];
    }
    const float mean_t = sum_t / (float)n;
    const float mean_w = sum_w / (float)n;

    float num = 0.0f;
    float den = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        const int idx = (start + i) % cap;
        const float ti = (tbuf_[idx] > t0) ? ((tbuf_[idx] - t0) / 1000.0f) : 0.0f;
        const float dt = ti - mean_t;
        const float dw = wbuf_[idx] - mean_w;
        num += dt * dw;
        den += dt * dt;
    }
    if (den < 1e-6f) return false;

    out_slope_gps = num / den;
    return isfinite(out_slope_gps);
}

void FlowMeter::update(uint32_t now_ms, float weight_g, bool shot_active) {
    st_.weight_raw_g = weight_g;

    // Update median buffer
    w1_ = w2_;
    w2_ = w3_;
    w3_ = weight_g;

    const float w_med = median3_(w1_, w2_, w3_);

    // Initialize filt on first valid update
    if (!isfinite(st_.weight_filt_g)) {
        st_.weight_filt_g = w_med;
        prev_w_filt_ = st_.weight_filt_g;
        push_sample_(now_ms, st_.weight_filt_g);
        prev_ms_ = now_ms;
        return;
    }

    // Filter weight (EMA)
    st_.weight_filt_g = ema_(st_.weight_filt_g, w_med, clampf_(cfg_.weight_ema_alpha, 0.0f, 1.0f));

    // Always keep regression buffer warm
    push_sample_(now_ms, st_.weight_filt_g);

    // If shot not active: keep estimator warm but mark invalid and do not integrate.
    if (!shot_active) {
        st_.flow_inst_gps = 0.0f;
        st_.flow_ema_gps = 0.0f;
        st_.flow_mlps = 0.0f;
        st_.flow_valid = false;
        st_.flow_start_ms = 0;
        above_ms_ = below_ms_ = 0;
        prev_w_filt_ = st_.weight_filt_g;
        prev_ms_ = now_ms;
        return;
    }

    if (prev_ms_ == 0) {
        prev_w_filt_ = st_.weight_filt_g;
        prev_ms_ = now_ms;
        return;
    }

    const uint32_t dt_ms = now_ms - prev_ms_;
    const float dt_s = dt_ms / 1000.0f;
    if (!(dt_s > cfg_.min_dt_s && dt_s < cfg_.max_dt_s)) {
        prev_w_filt_ = st_.weight_filt_g;
        prev_ms_ = now_ms;
        return;
    }

    // Compute robust flow slope over a window (cancels symmetric vibration)
    float slope_gps = st_.flow_inst_gps;
    if (!compute_slope_gps_(slope_gps)) {
        // Fallback: 2-point slope over this tick
        slope_gps = (dt_s > 0.0001f) ? ((st_.weight_filt_g - prev_w_filt_) / dt_s) : 0.0f;
    }

    slope_gps = clampf_(slope_gps, cfg_.flow_min_gps, cfg_.flow_max_gps);
    st_.flow_inst_gps = slope_gps;

    // Smooth flow
    st_.flow_ema_gps = ema_(st_.flow_ema_gps, slope_gps, clampf_(cfg_.flow_ema_alpha, 0.0f, 1.0f));

    // Variance (EWMA on squared deviation)
    const float err = st_.flow_ema_gps - st_.flow_inst_gps;
    st_.flow_var = ema_(st_.flow_var, err * err, clampf_(cfg_.var_alpha, 0.0f, 1.0f));
    st_.flow_std = sqrtf(fmaxf(0.0f, st_.flow_var));

    // Validity gating (debounced)
    if (st_.flow_valid) {
        if (st_.flow_ema_gps < cfg_.flow_stop_th_gps) {
            below_ms_ += dt_ms;
            if (below_ms_ >= cfg_.flow_stop_debounce_ms) {
                st_.flow_valid = false;
                st_.flow_start_ms = 0;
                above_ms_ = below_ms_ = 0;
            }
        } else {
            below_ms_ = 0;
        }
    } else {
        if (st_.flow_ema_gps > cfg_.flow_start_th_gps) {
            above_ms_ += dt_ms;
            if (above_ms_ >= cfg_.flow_start_debounce_ms) {
                st_.flow_valid = true;
                st_.flow_start_ms = now_ms;
                above_ms_ = below_ms_ = 0;
            }
        } else {
            above_ms_ = 0;
        }
    }

    // Total and averages (relative to tare)
    st_.total_g = fmaxf(0.0f, st_.weight_filt_g - tare_g_);
    const float t_shot_s = (st_.flow_start_ms > 0) ? ((now_ms - st_.flow_start_ms) / 1000.0f) : 0.0f;
    if (st_.flow_valid && t_shot_s > 0.5f) {
        st_.avg_flow_gps = st_.total_g / t_shot_s;
    } else {
        st_.avg_flow_gps = 0.0f;
    }

    // Convert to ml/s
    const float dens = (cfg_.density_g_per_ml > 0.5f) ? cfg_.density_g_per_ml : 1.0f;
    st_.flow_mlps = st_.flow_ema_gps / dens;

    // Update memories
    prev_w_filt_ = st_.weight_filt_g;
    prev_ms_ = now_ms;
}
