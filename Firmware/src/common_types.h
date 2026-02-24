#pragma once
#include "utils/compat.h"

// Number of shot profiles supported by the firmware.
// Keep this in sync with ProfileStore::_count.
#ifndef MAX_PROFILES
#define MAX_PROFILES 1
#endif

enum SystemState {
    STATE_BOOT,
    STATE_IDLE,
    STATE_BREW_PREHEAT,
    STATE_BREW_PREINFUSE,
    STATE_BREW_HOLD,        // bloom/soak — was incorrectly after FLOW
    STATE_BREW_FLOW,        // main extraction
    STATE_BREW_DRAIN,
    STATE_STEAM_WARMUP,
    STATE_STEAM_READY,
    STATE_FLUSH,
    STATE_DESCALE,
    STATE_FAULT
};

enum FaultCode {
    FAULT_NONE = 0,
    FAULT_TC_DISCONNECTED,
    FAULT_OVERTEMP,
    FAULT_ZC_MISSING,
    FAULT_SENSOR_TIMEOUT,
    FAULT_BOOT_RESET,
    FAULT_SCALE_ERROR,
    FAULT_DREAMSTEAM_OVERFILL,
    FAULT_PID_SATURATION
    ,FAULT_FLOW_STALL,
    FAULT_HEATER_STUCK_ON
};

// Fault policy classification.
// - RECOVERABLE: may auto-clear once inputs are healthy for a short period.
// - LATCHED: requires explicit user ACK (manual reset) even if inputs recover.
enum FaultClass : uint8_t {
    FAULTCLASS_RECOVERABLE = 0,
    FAULTCLASS_LATCHED = 1
};

static inline FaultClass fault_class(FaultCode fc) {
    switch (fc) {
        case FAULT_TC_DISCONNECTED:
        case FAULT_OVERTEMP:
        case FAULT_HEATER_STUCK_ON:
        case FAULT_BOOT_RESET:
            return FAULTCLASS_LATCHED;
        default:
            return FAULTCLASS_RECOVERABLE;
    }
}

static inline bool fault_is_latched(FaultCode fc) {
    return fault_class(fc) == FAULTCLASS_LATCHED;
}

// Roast classification used for grind advice + profile tuning.
enum RoastType : uint8_t {
    ROAST_LIGHT = 0,
    ROAST_MEDIUM = 1,
    ROAST_DARK = 2
};

// How a shot ended.
enum ShotStopReason : uint8_t {
    STOP_REASON_NONE = 0,
    STOP_REASON_TIME_CAP,
    STOP_REASON_WEIGHT_TARGET,
    STOP_REASON_USER_STOP,
    STOP_REASON_FAULT,
    STOP_REASON_INTERRUPTED
};

// Enhanced state snapshot with controller diagnostics
struct StateSnapshot {
    SystemState state = STATE_BOOT;
    FaultCode fault = FAULT_NONE;

    // Sensor readings
    float temp_c = 0.0f;
    float pressure_bar = 0.0f;
    float weight_g = 0.0f;
    float flow_rate = 0.0f;
    float flow_std_g_s = 0.0f;

    // Derived extraction metrics (best-effort; may be 0 if scale missing)
    float yield_g = 0.0f;
    float target_yield_g = 0.0f;
    float weight_rate_g_s = 0.0f;

    // ETA helpers (ms). Only populated when relevant.
    uint32_t eta_to_brew_ready_ms = 0;
    uint32_t eta_to_steam_ready_ms = 0;
    uint32_t eta_to_shot_end_ms = 0;
    uint32_t shot_total_cap_ms = 0;

    // Settings
    float brew_setpoint_c = 93.0f;
    float steam_setpoint_c = 145.0f;
    float pump_manual = 0.0f;
    uint16_t grind_level = 0;      // user-entered grinder dial level
    RoastType roast_type = ROAST_MEDIUM;
    bool wife_mode = false;

    // Advisor output (per profile × roast)
    uint16_t suggested_grind_level = 0;
    uint8_t suggested_grind_confidence = 0; // 0..100
    int8_t suggested_grind_delta = 0;       // +coarser / -finer, relative hint
    
    // Timing
    uint32_t shot_ms = 0;
    uint32_t extraction_ms = 0;
    uint32_t phase_ms = 0;
    uint32_t timestamp = 0;

    // Network status
    bool wifi_connected = false;
    bool shelly_latched = false;
    int8_t wifi_rssi = 0;

    // Profile info
    uint8_t active_profile_id = 0;
    
    // Shot statistics
    float shot_weight_start_g = 0.0f;
    float shot_weight_final_g = 0.0f;
    float shot_avg_temp_c = 0.0f;

    // Last shot summary (for a short "card" on the UI)
    bool last_shot_valid = false;
    uint8_t last_shot_profile_id = 0;
    ShotStopReason last_shot_stop_reason = STOP_REASON_NONE;
    float last_shot_target_yield_g = 0.0f;
    float last_shot_yield_g = 0.0f;
    float last_shot_overshoot_g = 0.0f;
    uint32_t last_shot_time_ms = 0;
    uint32_t last_shot_seq = 0; // increments each completed shot
    uint16_t last_shot_grind_level = 0;
    RoastType last_shot_roast_type = ROAST_MEDIUM;
    
    // Controller diagnostics
    float pid_output = 0.0f;
    float pid_p_term = 0.0f;
    float pid_i_term = 0.0f;
    float pid_d_term = 0.0f;
    
    // DreamSteam diagnostics
    bool dreamsteam_active = false;
    uint8_t dreamsteam_mode = 0; // DreamSteamController::Mode
    bool steam_load_detected = false;
    float steam_temp_drop_rate = 0.0f;
    uint8_t steam_pulse_power = 0;
    uint32_t steam_cumulative_ms = 0;
    float steam_effectiveness = 0.0f;

    // Calibration / diagnostics
    float scale_factor = 0.0f;
};

// Commands from UI to Core
struct UserCommand {
    enum Type : uint8_t {
        CMD_NONE = 0,
        CMD_START_BREW,
        CMD_STOP,
        CMD_ENTER_STEAM,
        CMD_EXIT_STEAM,
        CMD_FLUSH,
        CMD_DESCALE,
        CMD_TARE_SCALE,
        CMD_ACK_FAULT,
        CMD_SET_BREW_TEMP,
        CMD_SET_STEAM_TEMP,
        CMD_SET_PUMP_MANUAL,
        CMD_SELECT_PROFILE,
        CMD_SAVE_PROFILE,
        CMD_WIFI_RECONNECT,
        CMD_SHELLY_TOGGLE,
        CMD_SET_PID_PARAMS,
        CMD_SET_DREAMSTEAM_MODE,
        CMD_TOGGLE_DREAMSTEAM,
        CMD_RESET_CONTROLLERS,

        // Maintenance
        CMD_CLEAR_SHOT_HISTORY,
        CMD_SET_SCALE_CALIBRATION,
        CMD_TEST_PUMP,
        CMD_TEST_SOLENOID,

        // User inputs for learning/advisor
        CMD_SET_GRIND_LEVEL,
        CMD_SET_ROAST_TYPE,
        CMD_SET_WIFE_MODE
    } type = CMD_NONE;

    float param_f = 0.0f;
    uint32_t param_u32 = 0;
    int profile_id = 0;
    
    // Extended parameters for controller tuning
    float pid_kp = 0.0f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;
};

// Network commands sent from Core to Network task
struct NetCommand {
    enum Type : uint8_t {
        NET_NONE = 0,
        NET_SET_SHELLY,
        NET_WIFI_RECONNECT,
        NET_BLE_PROVISION_START,
        NET_OTA_CHECK_AND_UPDATE
    } type = NET_NONE;

    bool shelly_on = false;
};

// Enhanced shot profile with pressure profiling support
struct ShotProfile {
    char name[16] = "UNNAMED";
    float brew_temp_c = 93.0f;
    
    // Preinfusion
    uint32_t preinfuse_ms = 3000;
    float preinfuse_pump = 0.60f;     // was 0.45 — too low for vibe pump to wet puck
    
    // Bloom
    uint32_t bloom_ms = 2000;
    float bloom_pump = 0.0f;
    
    // Main extraction
    uint32_t brew_ms = 25000;
    float brew_pump = 1.0f;           // was 0.75 — vibe pump needs 100% for 9 bar

    // Target-based stop (0 = disabled)
    float target_yield_g = 0.0f;
    float stop_early_g = 1.5f;        // minimum early-stop to compensate drip
    uint16_t stop_lag_ms = 400;       // latency/in-flight window

    // Flow shaping (ramp + decline)
    bool enable_flow_shaping = false;  // was true — DISABLED for simplicity, enable later when pump confirmed working
    uint16_t flow_ramp_up_ms = 3000;
    uint16_t flow_ramp_down_ms = 3000;
    float flow_end_pump = 0.55f;      // fraction (0..1) of brew_pump at the very end

    // Temperature compensation during flow
    float flow_temp_offset_c = 2.0f;  // added to brew_temp_c while pump is running
    
    // Advanced: Pressure profiling
    bool enable_pressure_profile = false;
    float pressure_target_bar = 9.0f;
    
    // Flow profiling
    bool enable_flow_profile = false;
    float flow_target_ml_s = 2.0f;
};

struct ShotHistory {
    uint32_t timestamp = 0;
    uint8_t profile_id = 0;
    float brew_temp_c = 0.0f;
    float weight_start_g = 0.0f;
    float weight_final_g = 0.0f;
    float yield_g = 0.0f;
    float target_yield_g = 0.0f;
    float overshoot_g = 0.0f;
    uint32_t total_time_ms = 0;
    uint32_t preinfuse_ms = 0;
    uint32_t bloom_ms = 0;
    uint32_t brew_ms = 0;
    float avg_temp_c = 0.0f;
    float peak_temp_c = 0.0f;

    // User/context for learning
    uint16_t grind_level = 0;
    RoastType roast_type = ROAST_MEDIUM;
    ShotStopReason stop_reason = STOP_REASON_NONE;
    bool interrupted = false;
    
    // Advanced metrics
    float temp_stability_score = 0.0f;  // 0-100, higher is better
    float extraction_consistency = 0.0f; // 0-100
};