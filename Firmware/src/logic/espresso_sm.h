#pragma once

#include "../common_types.h"

// A small, testable state-machine model.
// This is intentionally decoupled from HAL/LVGL so we can unit-test
// the phase transitions and safety gating on desktop.

struct SmOutputs {
    uint8_t heater_duty = 0; // 0..100
    uint8_t pump_percent = 0; // 0..100
    bool solenoid_open = false;
};

class EspressoSM {
public:
    void reset() {
        state_ = STATE_IDLE;
        fault_ = FAULT_NONE;
        state_enter_ms_ = 0;
    }

    SystemState state() const { return state_; }
    FaultCode fault() const { return fault_; }
    uint32_t state_enter_ms() const { return state_enter_ms_; }

    void start_brew(uint32_t now_ms) {
        if (state_ == STATE_IDLE) {
            enter(STATE_BREW_PREHEAT, now_ms);
        }
    }

    void stop(uint32_t now_ms) {
        enter(STATE_BREW_DRAIN, now_ms);
    }

    // Very small transition model: PREHEAT -> PREINFUSE -> HOLD -> FLOW -> DRAIN -> IDLE
    SmOutputs step(uint32_t now_ms, float temp_c, const ShotProfile& p, float brew_setpoint_c) {
        SmOutputs o;
        if (state_ == STATE_FAULT) return o;

        switch (state_) {
            case STATE_BREW_PREHEAT:
                o.heater_duty = 100;
                if (temp_c >= brew_setpoint_c - 1.0f) enter(STATE_BREW_PREINFUSE, now_ms);
                break;
            case STATE_BREW_PREINFUSE:
                o.solenoid_open = true;
                o.pump_percent = (uint8_t)(p.preinfuse_pump * 100.0f);
                if ((now_ms - state_enter_ms_) >= p.preinfuse_ms)
                    enter((p.bloom_ms > 0) ? STATE_BREW_HOLD : STATE_BREW_FLOW, now_ms);
                break;
            case STATE_BREW_HOLD:
                o.solenoid_open = true;
                o.pump_percent = (uint8_t)(p.bloom_pump * 100.0f);
                if ((now_ms - state_enter_ms_) >= p.bloom_ms) enter(STATE_BREW_FLOW, now_ms);
                break;
            case STATE_BREW_FLOW:
                o.solenoid_open = true;
                o.pump_percent = (uint8_t)(p.brew_pump * 100.0f);
                if ((now_ms - state_enter_ms_) >= p.brew_ms) enter(STATE_BREW_DRAIN, now_ms);
                break;
            case STATE_BREW_DRAIN:
                o.solenoid_open = true;
                if ((now_ms - state_enter_ms_) >= 800) enter(STATE_IDLE, now_ms);
                break;
            case STATE_IDLE:
            default:
                break;
        }
        return o;
    }

private:
    void enter(SystemState s, uint32_t now_ms) {
        state_ = s;
        state_enter_ms_ = now_ms;
    }

    SystemState state_ = STATE_IDLE;
    FaultCode fault_ = FAULT_NONE;
    uint32_t state_enter_ms_ = 0;
};
