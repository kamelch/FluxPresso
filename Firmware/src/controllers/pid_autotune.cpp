
#include "pid_autotune.h"

void PIDAutoTune::begin(float setpoint_c, uint32_t now_ms) {
    active_ = true;
    done_ = false;
    res_ = Result{};
    sp_ = setpoint_c;
    start_ms_ = now_ms;
    high_ = true;
    out_ = 50.0f + cfg_.relay_amp;
    last_pv_ = NAN;
    last_der_ = 0;
    last_ms_ = now_ms;
    max_pv_ = -1e9f;
    min_pv_ = 1e9f;
    last_cross_ms_ = 0;
    periods_sum_ms_ = 0;
    cycles_ = 0;
}

void PIDAutoTune::cancel() {
    active_ = false;
    done_ = true;
    res_ = Result{};
    res_.success = false;
}

float PIDAutoTune::step(float pv_c, uint32_t now_ms) {
    if (!active_) return 0.0f;

    if (now_ms - start_ms_ > cfg_.max_ms) {
        cancel();
        return 0.0f;
    }

    // Relay with hysteresis around setpoint
    if (high_) {
        if (pv_c > sp_ + cfg_.hysteresis) {
            high_ = false;
            out_ = 50.0f - cfg_.relay_amp;
            // crossing
            if (last_cross_ms_ > 0 && (now_ms - last_cross_ms_) > 500) {
                periods_sum_ms_ += (now_ms - last_cross_ms_);
                cycles_++;
            }
            last_cross_ms_ = now_ms;
        }
    } else {
        if (pv_c < sp_ - cfg_.hysteresis) {
            high_ = true;
            out_ = 50.0f + cfg_.relay_amp;
        }
    }

    // Track extrema after settling
    if (now_ms - start_ms_ > cfg_.settle_ms) {
        if (pv_c > max_pv_) max_pv_ = pv_c;
        if (pv_c < min_pv_) min_pv_ = pv_c;
    }

    if (cycles_ >= cfg_.cycles_needed) {
        const float amp = (max_pv_ - min_pv_) / 2.0f;
        const float period_s = (periods_sum_ms_ / (float)cycles_) / 1000.0f;
        finalize_(amp, period_s);
        active_ = false;
        done_ = true;
    }

    // clamp output
    if (out_ < 0) out_ = 0;
    if (out_ > 100) out_ = 100;
    return out_;
}

void PIDAutoTune::finalize_(float amp, float period_s) {
    if (!(amp > 0.05f && period_s > 0.5f)) {
        res_.success = false;
        return;
    }
    // Relay method approximation for ultimate gain
    // Ku = (4*d)/(pi*a)
    const float d = cfg_.relay_amp;
    const float ku = (4.0f * d) / (3.1415926f * amp);
    const float tu = period_s;

    if (cfg_.use_tyreus_luyben) {
        // Tyreus–Luyben (more conservative than ZN)
        res_.kp = 0.454f * ku;
        res_.ki = res_.kp / (2.2f * tu);
        res_.kd = res_.kp * (tu / 6.3f);
    } else {
        // Ziegler–Nichols
        res_.kp = 0.6f * ku;
        res_.ki = 1.2f * ku / tu;
        res_.kd = 0.075f * ku * tu;
    }
    res_.success = true;
}
