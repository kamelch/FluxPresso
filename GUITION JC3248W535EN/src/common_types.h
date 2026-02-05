#pragma once
#include "utils/compat.h"

// Number of shot profiles supported by the firmware.
// Keep this in sync with ProfileStore::_count.
#ifndef MAX_PROFILES
#define MAX_PROFILES 6
#endif

enum SystemState {
    STATE_BOOT,
    STATE_IDLE,
    STATE_BREW_PREHEAT,
    STATE_BREW_PREINFUSE,
    STATE_BREW_FLOW,
    STATE_BREW_HOLD,
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

    // Settings
    float brew_setpoint_c = 93.0f;
    float steam_setpoint_c = 145.0f;
    float pump_manual = 0.0f;
    
    // Timing
    uint32_t shot_ms = 0;
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
    
    // Controller diagnostics
    float pid_output = 0.0f;
    float pid_p_term = 0.0f;
    float pid_i_term = 0.0f;
    float pid_d_term = 0.0f;
    
    // DreamSteam diagnostics
    bool dreamsteam_active = false;
    bool steam_load_detected = false;
    float steam_temp_drop_rate = 0.0f;
    uint8_t steam_pulse_power = 0;
    uint32_t steam_cumulative_ms = 0;
    float steam_effectiveness = 0.0f;
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
        CMD_RESET_CONTROLLERS
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
        NET_WIFI_RECONNECT
    } type = NET_NONE;

    bool shelly_on = false;
};

// Enhanced shot profile with pressure profiling support
struct ShotProfile {
    char name[16] = "UNNAMED";
    float brew_temp_c = 93.0f;
    
    // Preinfusion
    uint32_t preinfuse_ms = 3000;
    float preinfuse_pump = 0.45f;
    
    // Bloom
    uint32_t bloom_ms = 2000;
    float bloom_pump = 0.0f;
    
    // Main extraction
    uint32_t brew_ms = 25000;
    float brew_pump = 0.75f;
    
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
    uint32_t total_time_ms = 0;
    uint32_t preinfuse_ms = 0;
    uint32_t bloom_ms = 0;
    uint32_t brew_ms = 0;
    float avg_temp_c = 0.0f;
    float peak_temp_c = 0.0f;
    
    // Advanced metrics
    float temp_stability_score = 0.0f;  // 0-100, higher is better
    float extraction_consistency = 0.0f; // 0-100
};
