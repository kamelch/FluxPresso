
#pragma once
#include "../utils/compat.h"

class ShotPhaseDetector {
public:
    enum Phase : uint8_t { PREINFUSION=0, EXTRACTION=1, FINISHED=2 };

    struct Config {
        float flow_start_th_mlps = 0.5f;
        uint32_t debounce_ms = 350;
    };

    ShotPhaseDetector() : cfg_() {}
    explicit ShotPhaseDetector(const Config& cfg) : cfg_(cfg) {}

    void reset() {
        phase_ = PREINFUSION;
        extraction_start_ms_ = 0;
        acc_ms_ = 0;
    }

    void update(uint32_t now_ms, bool shot_active, bool flow_valid, float flow_mlps) {
        if (!shot_active) {
            reset();
            return;
        }
        if (phase_ == PREINFUSION) {
            if (flow_valid && flow_mlps > cfg_.flow_start_th_mlps) {
                acc_ms_ += step_ms_;
                if (acc_ms_ >= cfg_.debounce_ms) {
                    phase_ = EXTRACTION;
                    extraction_start_ms_ = now_ms;
                }
            } else {
                acc_ms_ = 0;
            }
        }
    }

    Phase phase() const { return phase_; }
    uint32_t extraction_start_ms() const { return extraction_start_ms_; }
    uint32_t extraction_ms(uint32_t now_ms) const {
        if (phase_ != EXTRACTION || extraction_start_ms_ == 0) return 0;
        return now_ms - extraction_start_ms_;
    }

    void set_step_ms(uint32_t step_ms){ step_ms_ = step_ms; }

private:
    Config cfg_;
    Phase phase_ = PREINFUSION;
    uint32_t extraction_start_ms_ = 0;
    uint32_t acc_ms_ = 0;
    uint32_t step_ms_ = 50;
};
