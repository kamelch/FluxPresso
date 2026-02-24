
#pragma once
#include "../utils/compat.h"
#include <math.h>

class PIDAutoTune {
public:
    struct Result { float kp=0, ki=0, kd=0; bool success=false; };

    struct Config {
        float relay_amp = 25.0f;     // % heater swing
        float hysteresis = 0.3f;     // °C band
        uint32_t settle_ms = 30000;  // wait before measuring
        uint32_t max_ms = 240000;    // 4 minutes
        uint8_t cycles_needed = 6;   // oscillation cycles
        bool use_tyreus_luyben = true;
    };

    PIDAutoTune() : cfg_() {}
    explicit PIDAutoTune(const Config& cfg) : cfg_(cfg) {}

    void begin(float setpoint_c, uint32_t now_ms);
    void cancel();
    bool active() const { return active_; }

    // Call at fixed rate; returns heater command 0..100
    float step(float pv_c, uint32_t now_ms);

    bool done() const { return done_; }
    Result result() const { return res_; }

private:
    Config cfg_;
    bool active_=false, done_=false;
    float sp_=0;
    uint32_t start_ms_=0;

    // Relay output state
    bool high_=true;
    float out_=0;

    // Peak detection
    float last_pv_=NAN;
    float last_der_=0;
    uint32_t last_ms_=0;

    // We store peaks to compute period/amplitude
    float max_pv_=-1e9f, min_pv_=1e9f;
    uint32_t last_cross_ms_=0;
    uint32_t periods_sum_ms_=0;
    uint8_t cycles_=0;

    Result res_{};

    void finalize_(float amp, float period_s);
};
