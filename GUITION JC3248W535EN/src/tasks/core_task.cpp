#include "common_types.h"
#include "project_config.h"
#include "profile_store.h"
#include "hal.h"
#include "controllers/pid_controller.h"
#include "controllers/dreamsteam_controller.h"
#include "utils/temp_predictor.h"
#include "utils/sensor_filters.h"
#include "utils/pump_ramp.h"
#include <WiFi.h>

#include "../diagnostics.h"
#include "../logic/safety.h"

#include "utils/shot_history.h"

extern QueueHandle_t queue_to_ui;
extern QueueHandle_t queue_from_ui;
extern QueueHandle_t queue_to_net;

// ===================================================================
// PERSISTENT STORAGE
// ===================================================================
static ProfileStore g_profiles;

// ===================================================================
// SYSTEM STATE
// ===================================================================
static SystemState current_state = STATE_BOOT;
static FaultCode current_fault = FAULT_NONE;
static uint32_t g_recovery_ok_since_ms = 0;

static inline bool is_recoverable_fault(FaultCode f) {
    switch (f) {
        case FAULT_ZC_MISSING:
        case FAULT_SENSOR_TIMEOUT:
        case FAULT_SCALE_ERROR:
        case FAULT_TC_DISCONNECTED:
            return true;
        default:
            return false;
    }
}

// ===================================================================
// TEMPERATURE CONTROLLERS
// ===================================================================
// Brew PID Controller
static PIDController::Config brew_pid_config = [](){
    PIDController::Config c;
    c.kp = BREW_PID_KP;
    c.ki = BREW_PID_KI;
    c.kd = BREW_PID_KD;
    c.derivative_filter = BREW_PID_DERIVATIVE_FILTER;
    c.output_min = BREW_PID_OUTPUT_MIN;
    c.output_max = BREW_PID_OUTPUT_MAX;
    c.setpoint_ramp_rate = BREW_PID_SETPOINT_RAMP;
    c.enable_anti_windup = true;
    return c;
}();
static PIDController brew_pid(brew_pid_config);

// Steam PID Controller
static PIDController::Config steam_pid_config = [](){
    PIDController::Config c;
    c.kp = STEAM_PID_KP;
    c.ki = STEAM_PID_KI;
    c.kd = STEAM_PID_KD;
    c.derivative_filter = STEAM_PID_DERIVATIVE_FILTER;
    c.output_min = STEAM_PID_OUTPUT_MIN;
    c.output_max = STEAM_PID_OUTPUT_MAX;
    c.setpoint_ramp_rate = STEAM_PID_SETPOINT_RAMP;
    c.enable_anti_windup = true;
    return c;
}();
static PIDController steam_pid(steam_pid_config);

// DreamSteam Controller
static DreamSteamController::Config dreamsteam_config = [](){
    DreamSteamController::Config c;
    c.mode = static_cast<DreamSteamController::Mode>(DREAMSTEAM_DEFAULT_MODE);
    c.temp_load_threshold_c = DREAMSTEAM_TEMP_LOAD_THRESHOLD;
    c.temp_drop_rate_threshold = DREAMSTEAM_TEMP_DROP_RATE_THRESHOLD;
    c.pulse_power_min = DREAMSTEAM_PULSE_POWER_MIN;
    c.pulse_power_max = DREAMSTEAM_PULSE_POWER_MAX;
    c.pulse_duration_ms = DREAMSTEAM_PULSE_DURATION_MS;
    c.pulse_interval_ms = DREAMSTEAM_PULSE_INTERVAL_MS;
    c.adaptive_gain = DREAMSTEAM_ADAPTIVE_GAIN;
    c.adaptive_window_ms = DREAMSTEAM_ADAPTIVE_WINDOW_MS;
    c.max_cumulative_pump_ms_per_min = DREAMSTEAM_MAX_PUMP_MS_PER_MIN;
    c.min_temp_for_refill_c = DREAMSTEAM_MIN_REFILL_TEMP;
    c.max_temp_for_refill_c = DREAMSTEAM_MAX_REFILL_TEMP;
    c.max_continuous_pulses = DREAMSTEAM_MAX_CONTINUOUS_PULSES;
    c.use_steam_valve_switch = DREAMSTEAM_USE_VALVE_SWITCH;
    c.steam_valve_pin = PIN_STEAM_VALVE;
    c.steam_valve_active_low = STEAM_VALVE_ACTIVE_LOW;
    return c;
}();
static DreamSteamController dreamsteam(dreamsteam_config);

// ===================================================================
// SENSOR FILTERS
// ===================================================================
#if ENABLE_SENSOR_FILTERING
#if USE_MEDIAN_FILTER
static MedianFilter3<float> temp_filter;
static MedianFilter3<float> weight_filter;
#else
static MovingAverageFilter<float, TEMP_FILTER_WINDOW> temp_filter;
static MovingAverageFilter<float, WEIGHT_FILTER_WINDOW> weight_filter;
#endif
#endif

// ===================================================================
// PUMP RAMPING
// ===================================================================
#if ENABLE_PUMP_RAMPING
static PumpRampController::Config pump_ramp_config = [](){
    PumpRampController::Config c;
    c.ramp_up_ms = PUMP_RAMP_UP_MS;
    c.ramp_down_ms = PUMP_RAMP_DOWN_MS;
    c.min_power_step = PUMP_MIN_STEP_PERCENT;
    c.enable_ramping = true;
    return c;
}();
static PumpRampController pump_ramp(pump_ramp_config);
#endif

// ===================================================================
// SHOT HISTORY
// ===================================================================
#if ENABLE_SHOT_PERSISTENCE
static ShotHistoryManager shot_history(MAX_SHOT_HISTORY);
#endif

// ===================================================================
// OVERCURRENT PROTECTION
// ===================================================================
#if ENABLE_OVERCURRENT_PROTECTION
static uint32_t heater_pump_overlap_start_ms = 0;
static bool heater_pump_both_active = false;
#endif

// ===================================================================
// SETPOINTS AND PROFILES
// ===================================================================
static float brew_setpoint_c = DEFAULT_BREW_TEMP_C;
static float steam_setpoint_c = DEFAULT_STEAM_TEMP_C;
static ShotProfile active_profile = {};
static uint8_t active_profile_id = 0;

// ===================================================================
// TIMING STATE
// ===================================================================
static uint32_t shot_start_ms = 0;
static uint32_t phase_start_ms = 0;

// ===================================================================
// MANUAL PUMP CONTROL
// ===================================================================
static float pump_manual = 0.0f;
static uint32_t g_manual_pump_start_ms = 0;
static uint32_t g_manual_pump_last_stop_ms = 0;
static bool g_manual_pump_was_running = false;
static bool g_manual_pump_warning_sent = false;

// ===================================================================
// SENSOR CACHE
// ===================================================================
static float g_last_temp_c = 0.0f;
static float g_last_weight_g = 0.0f;
static bool g_last_zc_ok = true;

// ===================================================================
// SHOT STATISTICS
// ===================================================================
static float shot_weight_start_g = 0.0f;
static float shot_temp_sum = 0.0f;
static float shot_temp_min = 999.0f;
static float shot_temp_max = -999.0f;
static uint32_t shot_temp_samples = 0;

// ===================================================================
// UI RATE LIMITING
// ===================================================================
static float last_ui_temp = 0.0f;
static float last_ui_weight = 0.0f;
static uint32_t last_ui_update_ms = 0;

// ===================================================================
// NETWORK STATE
// ===================================================================
static bool shelly_should_be_on = false;
static bool shelly_state_changed = false;

// ===================================================================
// TEMPERATURE PREDICTOR (OPTIONAL)
// ===================================================================
#if ENABLE_TEMP_PREDICTION
static TempPredictor temp_predictor;
#endif

// ===================================================================
// HARDWARE WATCHDOG (OPTIONAL)
// ===================================================================
#if ENABLE_HW_WATCHDOG
static hw_timer_t *watchdog_timer = NULL;
static volatile bool watchdog_kicked = false;

void IRAM_ATTR watchdog_isr() {
    if (!watchdog_kicked) {
        // Emergency shutdown
        digitalWrite(PIN_SSR, LOW);
        digitalWrite(PIN_AC_PWM, LOW);
        digitalWrite(PIN_SOLENOID, LOW);
        ESP.restart();
    }
    watchdog_kicked = false;
}
#endif

// ===================================================================
// STATE MACHINE HELPERS
// ===================================================================

/**
 * @brief Centralized state transition with entry/exit actions
 */
static inline void enter_state(SystemState next, uint32_t now_ms) {
    // Exit actions for current state
    switch (current_state) {
        case STATE_FLUSH:
        case STATE_DESCALE:
            pump_manual = 0.0f;
            break;
            
        case STATE_BREW_DRAIN: {
            // Shot complete - capture final statistics
            shot_temp_samples = (shot_temp_samples > 0) ? shot_temp_samples : 1;
            float shot_avg_temp = shot_temp_sum / shot_temp_samples;
            float shot_weight_final = g_last_weight_g;
            
            // Calculate temperature stability score (0-100)
            float temp_range = shot_temp_max - shot_temp_min;
            float stability_score = 100.0f * (1.0f - constrain(temp_range / 10.0f, 0.0f, 1.0f));
            
            #if ENABLE_SHOT_PERSISTENCE
            // Save to persistent storage
            ShotHistory history;
            history.timestamp = now_ms / 1000;  // Convert to seconds
            history.profile_id = active_profile_id;
            history.brew_temp_c = active_profile.brew_temp_c;
            history.weight_start_g = shot_weight_start_g;
            history.weight_final_g = shot_weight_final;
            history.yield_g = shot_weight_final - shot_weight_start_g;
            history.total_time_ms = now_ms - shot_start_ms;
            history.preinfuse_ms = active_profile.preinfuse_ms;
            history.bloom_ms = active_profile.bloom_ms;
            history.brew_ms = active_profile.brew_ms;
            history.avg_temp_c = shot_avg_temp;
            history.peak_temp_c = shot_temp_max;
            history.temp_stability_score = stability_score;
            
            shot_history.saveShot(history);
            
            // Print statistics
            Serial.printf("Shot completed: %.1fg in %.1fs, Temp: %.1f°C (±%.1f°C), Stability: %.0f%%\n",
                         history.yield_g, history.total_time_ms / 1000.0f,
                         history.avg_temp_c, temp_range, stability_score);
            #endif
            break;
        }
        
        case STATE_STEAM_READY:
            // Reset DreamSteam controller when exiting steam mode
            dreamsteam.reset();
            break;
            
        default:
            break;
    }

    current_state = next;
    phase_start_ms = now_ms;

    // Entry actions for new state
    switch (next) {
        case STATE_BREW_PREINFUSE:
            shot_start_ms = now_ms;
            pump_manual = 0.0f;
            shot_weight_start_g = g_last_weight_g;
            shot_temp_sum = 0.0f;
            shot_temp_min = 999.0f;
            shot_temp_max = -999.0f;
            shot_temp_samples = 0;
            
            // Ensure PID is ready for brewing
            brew_pid.setSetpoint(active_profile.brew_temp_c);
            
            #if SHELLY_ENABLED
            if (!shelly_should_be_on) {
                shelly_should_be_on = true;
                shelly_state_changed = true;
            }
            #endif
            break;
            
        case STATE_IDLE:
            pump_manual = 0.0f;
            
            // Switch to brew PID setpoint
            brew_pid.setSetpoint(brew_setpoint_c);
            
            #if SHELLY_ENABLED
            if (shelly_should_be_on) {
                shelly_should_be_on = false;
                shelly_state_changed = true;
            }
            #endif
            break;
            
        case STATE_BREW_PREHEAT:
            pump_manual = 0.0f;
            brew_pid.setSetpoint(brew_setpoint_c);
            break;
            
        case STATE_STEAM_WARMUP:
            // Reset and prepare steam PID
            steam_pid.reset();
            steam_pid.setSetpoint(steam_setpoint_c);
            dreamsteam.reset();
            break;
            
        case STATE_STEAM_READY:
            // DreamSteam is now active
            break;
            
        default:
            break;
    }
}

/**
 * @brief Process incoming commands from UI task
 */
static void apply_cmd(const UserCommand& cmd);
static void process_commands() {
    UserCommand cmd;
    while (xQueueReceive(queue_from_ui, &cmd, 0) == pdTRUE) {
        apply_cmd(cmd);
    }
}

static void apply_cmd(const UserCommand& cmd) {
    const uint32_t now = millis();
    
    switch(cmd.type) {
        case UserCommand::CMD_STOP:
            HAL_SetOutputs(0, false, 0);
            enter_state(STATE_IDLE, now);
            current_fault = FAULT_NONE;
            pump_manual = 0.0f;
            g_manual_pump_warning_sent = false;
            break;

        case UserCommand::CMD_ACK_FAULT:
            if (current_state == STATE_FAULT) {
                const bool tc_ok = (g_last_temp_c < 900.0f) && !isnan(g_last_temp_c);
                const bool temp_ok = (g_last_temp_c < 160.0f);
                const bool zc_ok = g_last_zc_ok;

                bool can_clear = false;
                switch (current_fault) {
                    case FAULT_TC_DISCONNECTED:
                        can_clear = tc_ok;
                        break;
                    case FAULT_OVERTEMP:
                        can_clear = tc_ok && temp_ok;
                        break;
                    case FAULT_ZC_MISSING:
                        can_clear = zc_ok;
                        break;
                    case FAULT_BOOT_RESET:
                    case FAULT_DREAMSTEAM_OVERFILL:
                    case FAULT_PID_SATURATION:
                        can_clear = true;
                        break;
                    default:
                        can_clear = tc_ok && temp_ok;
                        break;
                }

                if (can_clear) {
                    current_fault = FAULT_NONE;
                    enter_state(STATE_IDLE, now);
                }
            }
            break;

        case UserCommand::CMD_START_BREW:
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
                pump_manual = 0.0f;
                
                #if ENABLE_AUTO_TARE
                HAL_TareScale();
                vTaskDelay(pdMS_TO_TICKS(100));
                #endif
                
                if (g_last_temp_c < (brew_setpoint_c - PREHEAT_TEMP_OFFSET_C)) {
                    enter_state(STATE_BREW_PREHEAT, now);
                } else {
                    enter_state(STATE_BREW_PREINFUSE, now);
                }
            }
            break;

        case UserCommand::CMD_ENTER_STEAM:
            if (current_fault == FAULT_NONE) {
                enter_state(STATE_STEAM_WARMUP, now);
            }
            break;

        case UserCommand::CMD_EXIT_STEAM:
            if (current_state == STATE_STEAM_WARMUP || current_state == STATE_STEAM_READY) {
                enter_state(STATE_IDLE, now);
            }
            break;

        case UserCommand::CMD_FLUSH:
            if (current_state == STATE_IDLE) {
                enter_state(STATE_FLUSH, now);
            }
            break;

        case UserCommand::CMD_DESCALE:
            if (current_state == STATE_IDLE) {
                enter_state(STATE_DESCALE, now);
            }
            break;

        case UserCommand::CMD_TARE_SCALE:
            HAL_TareScale();
            break;

        case UserCommand::CMD_SET_BREW_TEMP:
            brew_setpoint_c = constrain(cmd.param_f, 85.0f, 105.0f);
            if (current_state == STATE_IDLE || current_state == STATE_BREW_PREHEAT) {
                brew_pid.setSetpoint(brew_setpoint_c);
            }
            break;

        case UserCommand::CMD_SET_STEAM_TEMP:
            steam_setpoint_c = constrain(cmd.param_f, 130.0f, 155.0f);
            if (current_state == STATE_STEAM_WARMUP || current_state == STATE_STEAM_READY) {
                steam_pid.setSetpoint(steam_setpoint_c);
            }
            break;

        case UserCommand::CMD_SET_PUMP_MANUAL:
            pump_manual = constrain(cmd.param_f, 0.0f, 1.0f);
            break;

        case UserCommand::CMD_SELECT_PROFILE:
            if (cmd.param_u32 < MAX_PROFILES) {
                active_profile_id = cmd.param_u32;
                if (g_profiles.load(active_profile_id, active_profile)) {
                    Serial.printf("Loaded profile %d: %s\n", active_profile_id, active_profile.name);
                }
            }
            break;

        case UserCommand::CMD_SAVE_PROFILE:
            if (cmd.param_u32 < MAX_PROFILES) {
                g_profiles.save(cmd.param_u32, active_profile);
                Serial.printf("Saved profile %d\n", cmd.param_u32);
            }
            break;

        case UserCommand::CMD_SET_PID_PARAMS: {
            // Update PID parameters in real-time
            auto config = brew_pid.getConfig();
            if (cmd.pid_kp > 0) config.kp = cmd.pid_kp;
            if (cmd.pid_ki >= 0) config.ki = cmd.pid_ki;
            if (cmd.pid_kd >= 0) config.kd = cmd.pid_kd;
            brew_pid.setConfig(config);
            Serial.printf("Updated PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", config.kp, config.ki, config.kd);
            break;
        }

        case UserCommand::CMD_SET_DREAMSTEAM_MODE: {
            auto config = dreamsteam.getConfig();
            config.mode = static_cast<DreamSteamController::Mode>(cmd.param_u32);
            dreamsteam.setConfig(config);
            Serial.printf("DreamSteam mode: %d\n", cmd.param_u32);
            break;
        }

        case UserCommand::CMD_RESET_CONTROLLERS:
            brew_pid.reset();
            steam_pid.reset();
            dreamsteam.reset();
            Serial.println("Controllers reset");
            break;

        case UserCommand::CMD_WIFI_RECONNECT: {
            NetCommand nc;
            nc.type = NetCommand::NET_WIFI_RECONNECT;
            xQueueSend(queue_to_net, &nc, 0);
            break;
        }

        case UserCommand::CMD_SHELLY_TOGGLE: {
            // Toggle desired Shelly state and inform network task
            shelly_should_be_on = !shelly_should_be_on;
            NetCommand nc;
            nc.type = NetCommand::NET_SET_SHELLY;
            nc.shelly_on = shelly_should_be_on;
            xQueueSend(queue_to_net, &nc, 0);
            break;
        }

        default:
            break;
    }
}

// ===================================================================
// CORE TASK MAIN LOOP
// ===================================================================

void core_task(void *pvParameters) {
    (void)pvParameters;

    Diag::init();
    
    // Initialize HAL
    if (!HAL_Init()) {
        Diag::fault(FAULT_BOOT_RESET, "HAL init failed");
        current_state = STATE_FAULT;
        current_fault = FAULT_BOOT_RESET;
    } else {
        Diag::log(LOG_INFO, "HAL initialized");

        #if ENABLE_HW_WATCHDOG
        watchdog_timer = timerBegin(1, 80, true); // Timer 1, prescaler 80 -> 1MHz
        timerAttachInterrupt(watchdog_timer, &watchdog_isr, true);
        timerAlarmWrite(watchdog_timer, (uint64_t)WATCHDOG_TIMEOUT_MS * 1000ULL, true);
        timerAlarmEnable(watchdog_timer);
        Diag::log(LOG_INFO, "Hardware watchdog enabled");
        #endif

        enter_state(STATE_IDLE, millis());
    }

    // Initialize default profile
    if (!g_profiles.load(0, active_profile)) {
        Diag::log(LOG_WARN, "No saved profile, using defaults");
        // active_profile already has defaults
    }

    #if ENABLE_SHOT_PERSISTENCE
    // Initialize shot history from NVS
    if (!shot_history.begin()) {
        Serial.println("WARNING: Shot history initialization failed");
    }
    #endif

    #if ENABLE_HW_WATCHDOG
    watchdog_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(watchdog_timer, &watchdog_isr, true);
    timerAlarmWrite(watchdog_timer, WATCHDOG_TIMEOUT_MS * 1000, true);
    timerAlarmEnable(watchdog_timer);
    watchdog_kicked = true;
    #endif

    // Main control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CORE_TASK_RATE_MS);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        const uint32_t now = millis();

        // === READ SENSORS ===
        float temp, weight;
        bool zc_ok;
        HAL_ReadSensors(temp, weight, zc_ok);
        
        // Apply filtering to reduce noise
        #if ENABLE_SENSOR_FILTERING
        temp = temp_filter.update(temp);
        weight = weight_filter.update(weight);
        #endif
        
        // Cache for command gating
        g_last_temp_c = temp;
        g_last_weight_g = weight;
        g_last_zc_ok = zc_ok;

        // === AUTO-RECOVERY (recoverable faults only) ===
        if (current_state == STATE_FAULT && current_fault != FAULT_NONE && is_recoverable_fault(current_fault)) {
            SafetyConfig cfg;
            cfg.max_temp_c = SAFETY_MAX_TEMP_C;
            SafetyInputs si{temp, weight, zc_ok};
            const bool conditions_ok = (safety_check(si, cfg) == FAULT_NONE);
            if (conditions_ok) {
                if (g_recovery_ok_since_ms == 0) g_recovery_ok_since_ms = now;
                if ((now - g_recovery_ok_since_ms) >= 3000) {
                    Diag::logf(LOG_WARN, "Auto-recovered from fault %d", (int)current_fault);
                    current_fault = FAULT_NONE;
                    g_recovery_ok_since_ms = 0;
                    enter_state(STATE_IDLE, now);
                }
            } else {
                g_recovery_ok_since_ms = 0;
            }
        } else {
            g_recovery_ok_since_ms = 0;
        }

        // === FAULT DETECTION (standardized) ===
        {
            SafetyConfig cfg;
            cfg.max_temp_c = SAFETY_MAX_TEMP_C;
            SafetyInputs si{temp, weight, zc_ok};
            FaultCode f = safety_check(si, cfg);
            if (f != FAULT_NONE) {
                enter_state(STATE_FAULT, now);
                current_fault = f;
                Diag::fault(f, "Safety fault triggered");
                HAL_SetOutputs(0, false, 0);

                StateSnapshot snap{};
                snap.state = STATE_FAULT;
                snap.fault = f;
                snap.temp_c = temp;
                snap.weight_g = weight;
                snap.timestamp = now;
                snap.wifi_connected = WiFi.isConnected();
                snap.wifi_rssi = snap.wifi_connected ? (int8_t)WiFi.RSSI() : 0;
                xQueueSend(queue_to_ui, &snap, 0);
                continue;
            }
        }

        // Process UI commands
        process_commands();

        // === STATE MACHINE ===
        uint8_t pump_percent = 0;
        bool solenoid_open = false;
        uint8_t heater_duty = 0;
        float pid_output = 0.0f;
        
        // PID diagnostics
        float pid_p = 0.0f, pid_i = 0.0f, pid_d = 0.0f;

        if (current_state == STATE_FAULT) {
            // Everything off in fault state
            heater_duty = 0;
            pump_percent = 0;
            solenoid_open = false;
        }
        else if (current_state == STATE_IDLE) {
            // Brew PID temperature control
            pid_output = brew_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            
            #if ENABLE_PID_DIAGNOSTICS
            const auto& config = brew_pid.getConfig();
            float error = brew_pid.getSetpoint() - temp;
            pid_p = config.kp * error;
            pid_i = brew_pid.getIntegral();
            pid_d = pid_output - pid_p - pid_i;
            #endif

            // Manual pump with safety checks
            const bool want_manual = (pump_manual > 0.01f);

            // Pressure relief after manual pump stops
            if (!want_manual && g_manual_pump_was_running && 
                (now - g_manual_pump_last_stop_ms) < MANUAL_DRAIN_TIME_MS) {
                pump_percent = 0;
                solenoid_open = true;
            }

            if (want_manual) {
                if (!g_manual_pump_was_running) {
                    g_manual_pump_start_ms = now;
                    g_manual_pump_was_running = true;
                    g_manual_pump_warning_sent = false;
                }

                const uint32_t manual_duration = now - g_manual_pump_start_ms;

                if (manual_duration > MANUAL_PUMP_WARNING_MS && !g_manual_pump_warning_sent) {
                    Diag::log(LOG_WARN, "Manual pump running >30s");
                    g_manual_pump_warning_sent = true;
                }

                if (manual_duration > MANUAL_PUMP_TIMEOUT_MS) {
                    pump_manual = 0.0f;
                    g_manual_pump_last_stop_ms = now;
                    g_manual_pump_was_running = false;
                    // Safety escalation: do not silently continue after a stuck manual pump.
                    enter_state(STATE_FAULT, now);
                    current_fault = FAULT_SENSOR_TIMEOUT;
                    Diag::fault(current_fault, "Manual pump timeout");
                } else if (temp >= MANUAL_PUMP_MAX_TEMP) {
                    pump_manual = 0.0f;
                    g_manual_pump_last_stop_ms = now;
                    g_manual_pump_was_running = false;
                    enter_state(STATE_FAULT, now);
                    current_fault = FAULT_OVERTEMP;
                    Diag::fault(current_fault, "Manual pump overtemp");
                } else if (!zc_ok) {
                    enter_state(STATE_FAULT, now);
                    current_fault = FAULT_ZC_MISSING;
                    pump_manual = 0.0f;
                    g_manual_pump_last_stop_ms = now;
                    g_manual_pump_was_running = false;
                    Diag::fault(current_fault, "Zero-cross missing during manual pump");
                } else {
                    pump_percent = (uint8_t)(pump_manual * 100.0f);
                    solenoid_open = true;
                }
            } else {
                if (g_manual_pump_was_running) {
                    g_manual_pump_last_stop_ms = now;
                    g_manual_pump_was_running = false;
                }
            }
        }
        else if (current_state == STATE_BREW_PREHEAT) {
            pid_output = brew_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            pump_percent = 0;
            solenoid_open = false;

            if (temp >= (brew_setpoint_c - 1.0f)) {
                enter_state(STATE_BREW_PREINFUSE, now);
            }
        }
        else if (current_state == STATE_BREW_PREINFUSE) {
            pid_output = brew_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            solenoid_open = true;
            pump_percent = (uint8_t)(active_profile.preinfuse_pump * 100.0f);

            // Collect statistics
            shot_temp_sum += temp;
            shot_temp_min = min(shot_temp_min, temp);
            shot_temp_max = max(shot_temp_max, temp);
            shot_temp_samples++;

            if ((now - phase_start_ms) >= active_profile.preinfuse_ms) {
                enter_state((active_profile.bloom_ms > 0) ? STATE_BREW_HOLD : STATE_BREW_FLOW, now);
            }
        }
        else if (current_state == STATE_BREW_HOLD) {
            pid_output = brew_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            solenoid_open = true;
            pump_percent = (uint8_t)(active_profile.bloom_pump * 100.0f);

            shot_temp_sum += temp;
            shot_temp_min = min(shot_temp_min, temp);
            shot_temp_max = max(shot_temp_max, temp);
            shot_temp_samples++;

            if ((now - phase_start_ms) >= active_profile.bloom_ms) {
                enter_state(STATE_BREW_FLOW, now);
            }
        }
        else if (current_state == STATE_BREW_FLOW) {
            pid_output = brew_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            solenoid_open = true;
            pump_percent = (uint8_t)(active_profile.brew_pump * 100.0f);

            shot_temp_sum += temp;
            shot_temp_min = min(shot_temp_min, temp);
            shot_temp_max = max(shot_temp_max, temp);
            shot_temp_samples++;

            if ((now - phase_start_ms) >= active_profile.brew_ms) {
                heater_duty = 0;
                pump_percent = 0;
                solenoid_open = true;
                enter_state(STATE_BREW_DRAIN, now);
            }
        }
        else if (current_state == STATE_BREW_DRAIN) {
            heater_duty = 0;
            pump_percent = 0;
            solenoid_open = true;

            if ((now - phase_start_ms) >= DRAIN_TIME_MS) {
                solenoid_open = false;
                enter_state(STATE_IDLE, now);
            }
        }
        else if (current_state == STATE_FLUSH) {
            heater_duty = 0;
            solenoid_open = true;
            pump_percent = 100;

            if ((now - phase_start_ms) >= FLUSH_TIME_MS) {
                enter_state(STATE_IDLE, now);
            }
        }
        else if (current_state == STATE_DESCALE) {
            heater_duty = 0;
            solenoid_open = (pump_manual > 0.01f);
            pump_percent = (uint8_t)(pump_manual * 100.0f);
        }
        else if (current_state == STATE_STEAM_WARMUP) {
            solenoid_open = false;
            pump_percent = 0;
            
            // Steam PID control
            pid_output = steam_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);

            if (temp >= steam_setpoint_c - 0.5f) {
                enter_state(STATE_STEAM_READY, now);
            }
        }
        else if (current_state == STATE_STEAM_READY) {
            solenoid_open = false;
            
            // Steam PID control for heater
            pid_output = steam_pid.update(temp, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            
            // DreamSteam pump assist
            pump_percent = dreamsteam.update(temp, steam_setpoint_c, now, zc_ok);
            
            #if ENABLE_PID_DIAGNOSTICS
            const auto& config = steam_pid.getConfig();
            float error = steam_pid.getSetpoint() - temp;
            pid_p = config.kp * error;
            pid_i = steam_pid.getIntegral();
            pid_d = pid_output - pid_p - pid_i;
            #endif
        }

        // === FINAL SAFETY INTERLOCK ===
        if (pump_percent > 0 && !zc_ok) {
            enter_state(STATE_FAULT, now);
            current_fault = FAULT_ZC_MISSING;
            pump_percent = 0;
            solenoid_open = false;
            heater_duty = 0;
        }

        // === PUMP RAMPING ===
        #if ENABLE_PUMP_RAMPING
        pump_percent = pump_ramp.setTarget(pump_percent, now);
        #endif

        // === OVERCURRENT PROTECTION ===
        #if ENABLE_OVERCURRENT_PROTECTION
        const bool heater_active = (heater_duty > 10);
        const bool pump_active = (pump_percent > 10);
        
        if (heater_active && pump_active) {
            if (!heater_pump_both_active) {
                heater_pump_both_active = true;
                heater_pump_overlap_start_ms = now;
            } else {
                const uint32_t overlap_time = now - heater_pump_overlap_start_ms;
                if (overlap_time > MAX_HEATER_PUMP_OVERLAP_MS) {
                    #if HEATER_PRIORITY_MODE
                    // Reduce pump to 50% when heater is on for too long
                    pump_percent = pump_percent / 2;
                    Serial.println("WARNING: Overcurrent protection - reducing pump");
                    #else
                    // Default: Reduce heater to 60% when both running
                    heater_duty = (heater_duty * 60) / 100;
                    Serial.println("WARNING: Overcurrent protection - reducing heater");
                    #endif
                }
            }
        } else {
            heater_pump_both_active = false;
        }
        #endif

        // Apply outputs
        HAL_SetOutputs(pump_percent, solenoid_open, heater_duty);

        // Send Shelly update if state changed
        #if SHELLY_ENABLED
        if (shelly_state_changed) {
            NetCommand nc;
            nc.type = NetCommand::NET_SET_SHELLY;
            nc.shelly_on = shelly_should_be_on;
            xQueueSend(queue_to_net, &nc, 0);
            shelly_state_changed = false;
        }
        #endif

        // === SEND STATE SNAPSHOT TO UI ===
        StateSnapshot snap{};
        snap.state = current_state;
        snap.fault = current_fault;
        snap.temp_c = temp;
        snap.weight_g = weight;
        snap.brew_setpoint_c = brew_setpoint_c;
        snap.steam_setpoint_c = steam_setpoint_c;
        snap.pump_manual = pump_manual;
        snap.timestamp = now;
        snap.shelly_latched = shelly_should_be_on;
        snap.active_profile_id = active_profile_id;

        if (current_state == STATE_BREW_PREINFUSE || 
            current_state == STATE_BREW_HOLD || 
            current_state == STATE_BREW_FLOW ||
            current_state == STATE_BREW_DRAIN) {
            snap.shot_ms = now - shot_start_ms;
            snap.phase_ms = now - phase_start_ms;
        } else {
            snap.shot_ms = 0;
            snap.phase_ms = 0;
        }

        // PID diagnostics
        #if ENABLE_PID_DIAGNOSTICS
        snap.pid_output = pid_output;
        snap.pid_p_term = pid_p;
        snap.pid_i_term = pid_i;
        snap.pid_d_term = pid_d;
        #endif

        // DreamSteam diagnostics
        if (current_state == STATE_STEAM_READY) {
            const auto& ds_state = dreamsteam.getState();
            snap.dreamsteam_active = true;
            snap.steam_load_detected = ds_state.steam_load_detected;
            snap.steam_temp_drop_rate = ds_state.temp_drop_rate;
            snap.steam_pulse_power = ds_state.current_pulse_power;
            snap.steam_cumulative_ms = ds_state.cumulative_pump_ms;
            snap.steam_effectiveness = ds_state.effectiveness_score;
        }

        // === UI RATE LIMITING ===
        // Only send updates if values changed significantly or force update timeout
        bool should_update = false;
        
        const float temp_delta = fabsf(temp - last_ui_temp);
        const float weight_delta = fabsf(weight - last_ui_weight);
        const uint32_t time_since_update = now - last_ui_update_ms;
        
        if (temp_delta >= UI_UPDATE_THRESHOLD_TEMP ||
            weight_delta >= UI_UPDATE_THRESHOLD_WEIGHT ||
            time_since_update >= UI_FORCE_UPDATE_MS ||
            current_state != STATE_IDLE) {  // Always update during brewing/steam
            should_update = true;
            last_ui_temp = temp;
            last_ui_weight = weight;
            last_ui_update_ms = now;
        }

        if (should_update) {
            if (xQueueSend(queue_to_ui, &snap, 0) != pdTRUE) {
                static uint32_t last_warning = 0;
                if (now - last_warning > 5000) {
                    Serial.println("WARNING: UI queue full!");
                    last_warning = now;
                }
            }
        }

        // Kick watchdog
        #if ENABLE_HW_WATCHDOG
        watchdog_kicked = true;
        #endif
    }
}
