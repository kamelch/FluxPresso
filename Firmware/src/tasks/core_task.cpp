// V9_TESTS_SAFE
#include "common_types.h"
#include "project_config.h"
#include "profile_store.h"
#include "hal.h"
#include "controllers/pid_controller.h"
#include "controllers/dreamsteam_controller.h"
#include "utils/temp_predictor.h"
#include "utils/sensor_filters.h"
#include "utils/pump_ramp.h"
#include "utils/flow_shaper.h"
#include "utils/flow_meter.h"
#include "utils/stable_weight.h"
#include "../core/shot_phase.h"
#include "../core/maintenance_tracker.h"
#include "controllers/pid_autotune.h"
#include "utils/grind_advisor.h"
#include <WiFi.h>
#include "driver/gpio.h"
#include "esp_task_wdt.h"

#include <Preferences.h>

#include "../diagnostics.h"
#include "../logic/safety.h"

#include "utils/shot_history.h"

#include "../core/fault_manager.h"

extern QueueHandle_t queue_to_ui;
extern QueueHandle_t queue_from_ui;
extern QueueHandle_t queue_to_net;


static inline bool weight_valid(float w) {
    return !isnan(w) && w > -900.0f && w < 5000.0f;
}

// ===================================================================
// PERSISTENT STORAGE
// ===================================================================
static ProfileStore g_profiles;
static Preferences g_settings;
static GrindAdvisor g_grind_advisor;

// ===================================================================
// SYSTEM STATE
// ===================================================================
static SystemState current_state = STATE_BOOT;
static FaultCode current_fault = FAULT_NONE;
static uint32_t g_preheat_ready_since_ms = 0;
static uint32_t g_recovery_ok_since_ms = 0;
static uint32_t g_stop_ack_since_ms = 0;  // separate from auto-recovery (which resets it every iteration for latched faults)
static uint32_t g_shelly_on_at_ms = 0;   // when Shelly was last turned ON (for stabilization delay)

static inline bool is_recoverable_fault(FaultCode f) { return FaultMgr::is_recoverable(f); }


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

// User-entered learning context
static uint16_t g_grind_level = 0;
static RoastType g_roast_type = ROAST_MEDIUM;
static bool g_wife_mode = false;

// Advisor output
static uint16_t g_suggested_grind = 0;
static uint8_t g_suggested_conf = 0;
static int8_t g_suggested_delta = 0;

static inline void refresh_grind_suggestion() {
    g_grind_advisor.suggest(active_profile_id, g_roast_type, g_grind_level,
                            g_suggested_grind, g_suggested_conf, g_suggested_delta);
}


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

// Weight/flow estimation
static FlowMeter g_flow_meter;
// UI-only weight stabilizer: hides "place-to-place" wiggle while still allowing
// small real shifts to be detected via rolling averages.
static StableWeight g_ui_weight;
static ShotPhaseDetector g_shot_phase;
static float g_weight_rate_ema_g_s = 0.0f;
 // kept for legacy uses (mirrors flow EMA)
static float g_prev_weight_g = 0.0f;
static uint32_t g_prev_weight_ms = 0;

// Flow profiling controller (pump trim). Output is trim in percent.
// Note: avoid designated initializers for broad compiler compatibility.
static PIDController g_flow_pid;
static bool g_flow_pid_inited = false;

static void init_flow_pid_once_() {
    if (g_flow_pid_inited) return;
    PIDController::Config cfg;
    cfg.kp = 12.0f;
    cfg.ki = 1.0f;
    cfg.kd = 0.0f;
    cfg.derivative_filter = 0.20f;
    // Output is a *trim* in pump percent applied around a feed-forward base.
    // Wider negative range allows reducing pump power for coarse grinds / low target flows.
    cfg.output_min = -60.0f;
    cfg.output_max = 25.0f;
    cfg.setpoint_ramp_rate = 0.0f;
    cfg.enable_anti_windup = true;
    g_flow_pid = PIDController(cfg);
    g_flow_pid_inited = true;
}
// Maintenance tracking
static MaintenanceTracker g_maintenance;


// Last-shot summary
static uint32_t g_shot_seq = 0;
static ShotStopReason g_last_stop_reason = STOP_REASON_NONE;

// Fault-time pressure relief & shot finalization (rare but important):
// If a fault occurs mid-shot, we vent briefly, then finalize the shot record safely.
static uint32_t g_fault_drain_until_ms = 0;
static bool g_fault_finalize_pending = false;
static uint32_t g_fault_finalize_at_ms = 0;

// Stop-early auto-tune (learn from overshoot; debounced to protect flash)
static bool g_stop_early_dirty = false;
static uint8_t g_stop_early_pending = 0;
static uint32_t g_stop_early_last_save_ms = 0;

static bool g_last_shot_valid = false;
static uint8_t g_last_shot_profile_id = 0;
static float g_last_shot_target_yield_g = 0.0f;
static float g_last_shot_yield_g = 0.0f;
static float g_last_shot_overshoot_g = 0.0f;
static uint32_t g_last_shot_time_ms = 0;
static uint16_t g_last_shot_grind_level = 0;
static RoastType g_last_shot_roast_type = ROAST_MEDIUM;

// Steam UX: temporary warmup boost to reduce "waiting" while still stabilizing.
static uint32_t g_steam_boost_until_ms = 0;

// ===================================================================
// UI RATE LIMITING
// ===================================================================
static float last_ui_temp = 0.0f;
static float last_ui_weight = 0.0f;
static uint32_t last_ui_update_ms = 0;

// ===================================================================
// NETWORK STATE
// ===================================================================
static bool shelly_should_be_on = true;
static bool shelly_state_changed = false;


// ===================================================================
// OUTPUT TEST MODE (from UI Maintenance screen)
// ===================================================================
static uint32_t g_test_pump_until_ms = 0;
static float    g_test_pump_power = 0.0f; // 0..1
static uint32_t g_test_solenoid_until_ms = 0;

// Flow stall detection
static float g_flow_last_yield_g = 0.0f;
static uint32_t g_flow_last_progress_ms = 0;

// Heater saturation fault tracking
static uint32_t g_pid_sat_start_ms = 0;
static float g_pid_sat_start_temp = 0.0f;

// Heater stuck-ON heuristic: if heater commanded OFF but temp rises quickly.
static uint32_t g_heater_off_start_ms = 0;
static float g_heater_off_start_temp = 0.0f;

// Persisted scale calibration (applied after HAL_Init)
static float g_saved_scale_factor = DEFAULT_SCALE_CALIBRATION;
static long  g_saved_scale_offset = (long)DEFAULT_SCALE_OFFSET;

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
        // Emergency shutdown (ISR-safe)
        gpio_set_level((gpio_num_t)PIN_SSR, 0);
        gpio_set_level((gpio_num_t)PIN_AC_PWM, 0);
        gpio_set_level((gpio_num_t)PIN_SOLENOID, 0);
        ESP.restart();
    }
    watchdog_kicked = false;
}
#endif


// ===================================================================
// SHOT FINALIZATION & LEARNING
// ===================================================================

static void maybe_autotune_stop_early(float overshoot_g, float yield_g, float time_s, uint32_t now_ms) {
    // Learn only from clean, weight-target shots.
    if (g_last_stop_reason != STOP_REASON_WEIGHT_TARGET) return;
    if (!(active_profile.target_yield_g > 0.1f)) return;
    if (!isfinite(overshoot_g) || !isfinite(yield_g) || !isfinite(time_s)) return;

    // Guard rails: ignore outliers / bad data
    if (time_s < 12.0f || time_s > 60.0f) return;
    if (yield_g < 2.0f || yield_g > 250.0f) return;
    if (fabsf(overshoot_g) > 8.0f) return;

    const float old = active_profile.stop_early_g;
    const float gain = 0.30f;                 // how aggressively we learn from overshoot
    float delta = overshoot_g * gain;         // +overshoot -> stop earlier next time
    delta = constrain(delta, -0.6f, 0.9f);    // step limit per shot

    float candidate = old + delta;
    candidate = constrain(candidate, 0.5f, 6.0f);

    // Small deadband avoids churn
    if (fabsf(candidate - old) < 0.05f) return;

    // Smooth the update (EWMA)
    active_profile.stop_early_g = 0.75f * old + 0.25f * candidate;

    g_stop_early_dirty = true;
    g_stop_early_pending = (g_stop_early_pending < 255) ? (uint8_t)(g_stop_early_pending + 1) : (uint8_t)255;

    const bool should_save = (g_stop_early_pending >= 3) ||
                             (g_stop_early_last_save_ms == 0) ||
                             ((now_ms - g_stop_early_last_save_ms) > 600000UL); // 10 min

    if (should_save) {
        g_profiles.save(active_profile_id, active_profile);
        g_stop_early_dirty = false;
        g_stop_early_pending = 0;
        g_stop_early_last_save_ms = now_ms;
        Diag::logf(LOG_INFO, "Auto-tuned stop_early to %.2fg (profile %u)",
                   active_profile.stop_early_g, (unsigned)active_profile_id);
    }
}

static void finalize_shot(uint32_t now_ms, ShotStopReason reason, float weight_final_g) {
    if (shot_start_ms == 0) return;

    // Stabilize stats even if sensors glitch
    shot_temp_samples = (shot_temp_samples > 0) ? shot_temp_samples : 1;
    const float shot_avg_temp = shot_temp_sum / (float)shot_temp_samples;

    const float shot_weight_final = weight_valid(weight_final_g) ? weight_final_g :
                                    (weight_valid(g_last_weight_g) ? g_last_weight_g : NAN);

    const float yield_g = (weight_valid(shot_weight_start_g) && weight_valid(shot_weight_final))
                            ? (shot_weight_final - shot_weight_start_g)
                            : NAN;

    // Temperature stability score (0-100)
    const float temp_range = shot_temp_max - shot_temp_min;
    const float stability_score = 100.0f * (1.0f - constrain(temp_range / 10.0f, 0.0f, 1.0f));

    const float target_yield_g = active_profile.target_yield_g;
    const float overshoot_g = (target_yield_g > 0.1f && isfinite(yield_g)) ? (yield_g - target_yield_g) : 0.0f;

    // Update "last shot" summary for UI
    g_shot_seq++;
    g_last_shot_valid = true;
    // Maintenance counters (persisted)
    g_maintenance.on_shot_complete(isfinite(yield_g) ? yield_g : 0.0f);

    g_last_shot_profile_id = active_profile_id;
    g_last_shot_target_yield_g = target_yield_g;
    g_last_shot_yield_g = isfinite(yield_g) ? yield_g : 0.0f;
    g_last_shot_overshoot_g = overshoot_g;
    g_last_shot_time_ms = now_ms - shot_start_ms;
    g_last_shot_grind_level = g_grind_level;
    g_last_shot_roast_type = g_roast_type;

    // Persist: shot not in progress anymore
    g_settings.putBool("shot_inprog", false);

    // Feed grind advisor (only when data is trustworthy; never learn from glitches/outliers)
    const float time_s = (now_ms - shot_start_ms) / 1000.0f;
    const bool yield_ok = weight_valid(shot_weight_start_g) && weight_valid(shot_weight_final) && (yield_g > 0.5f) && (yield_g < 300.0f);
    const bool time_ok = (time_s >= 12.0f) && (time_s <= 60.0f);
    const bool stop_ok = (reason == STOP_REASON_WEIGHT_TARGET) || (reason == STOP_REASON_TIME_CAP);

    if (yield_ok && time_ok && stop_ok) {
        g_grind_advisor.observe(active_profile_id, g_roast_type, g_grind_level,
                                yield_g, target_yield_g, time_s);
    }

    // Always refresh the suggestion for the next shot (even if we didn't learn)
    g_grind_advisor.suggest(active_profile_id, g_roast_type, g_grind_level,
                            g_suggested_grind, g_suggested_conf, g_suggested_delta);

    // Auto-tune stop-early from overshoot (only clean weight-target shots)
    if (yield_ok && time_ok && (reason == STOP_REASON_WEIGHT_TARGET)) {
        maybe_autotune_stop_early(overshoot_g, yield_g, time_s, now_ms);
    }

#if ENABLE_SHOT_PERSISTENCE
    ShotHistory history;
    history.timestamp = now_ms / 1000;  // seconds since boot (no RTC)
    history.profile_id = active_profile_id;
    history.brew_temp_c = active_profile.brew_temp_c;
    history.weight_start_g = shot_weight_start_g;
    history.weight_final_g = shot_weight_final;
    history.yield_g = isfinite(yield_g) ? yield_g : 0.0f;
    history.target_yield_g = target_yield_g;
    history.overshoot_g = overshoot_g;
    history.total_time_ms = now_ms - shot_start_ms;
    history.preinfuse_ms = active_profile.preinfuse_ms;
    history.bloom_ms = active_profile.bloom_ms;
    history.brew_ms = active_profile.brew_ms;
    history.avg_temp_c = shot_avg_temp;
    history.peak_temp_c = shot_temp_max;
    history.temp_stability_score = stability_score;
    history.grind_level = g_grind_level;
    history.roast_type = g_roast_type;
    history.stop_reason = reason;

    shot_history.saveShot(history);

    Serial.printf("Shot done (%s): %.1fg in %.1fs, Temp: %.1f°C, Stability: %.0f%%, Overshoot: %.1fg\n",
                  (reason == STOP_REASON_WEIGHT_TARGET) ? "weight" :
                  (reason == STOP_REASON_TIME_CAP) ? "time" :
                  (reason == STOP_REASON_USER_STOP) ? "user" :
                  (reason == STOP_REASON_FAULT) ? "fault" :
                  (reason == STOP_REASON_INTERRUPTED) ? "int" : "-",
                  history.yield_g, history.total_time_ms / 1000.0f,
                  history.avg_temp_c, stability_score, history.overshoot_g);
#endif

    // Clear shot_start marker (prevents accidental double-finalize)
    shot_start_ms = 0;
}

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
            // Shot complete - finalize and learn (includes drip after stop)
            finalize_shot(now_ms, g_last_stop_reason, g_last_weight_g);
            break;
        }

        case STATE_STEAM_READY:
            // Reset DreamSteam controller when exiting steam mode
            dreamsteam.reset();
            break;
            
        case STATE_FAULT:
            // Clear any transient fault-drain/finalize state when leaving FAULT
            g_fault_drain_until_ms = 0;
            g_fault_finalize_pending = false;
            g_fault_finalize_at_ms = 0;
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
            shot_weight_start_g = weight_valid(g_last_weight_g) ? g_last_weight_g : 0.0f;
            g_prev_weight_g = weight_valid(g_last_weight_g) ? g_last_weight_g : 0.0f;
            g_prev_weight_ms = now_ms;
            g_weight_rate_ema_g_s = 0.0f;
            g_flow_last_yield_g = 0.0f;
            g_flow_last_progress_ms = now_ms;
            g_last_stop_reason = STOP_REASON_NONE;
            shot_temp_sum = 0.0f;
            shot_temp_min = 999.0f;
            shot_temp_max = -999.0f;
            shot_temp_samples = 0;

            // Reset flow estimator so the new shot starts clean
            // (stale median buffer / EMA from previous shot would corrupt first readings)
            g_flow_meter.start_shot(now_ms, shot_weight_start_g);

            // Reset flow PID to prevent integral windup carryover between shots
            g_flow_pid.reset();
            
            // Ensure PID is ready for brewing
            brew_pid.setSetpoint(active_profile.brew_temp_c);

            // Reboot-safe marker
            g_settings.putBool("shot_inprog", true);
            g_settings.putUChar("shot_prof", active_profile_id);
            g_settings.putUShort("grind_lvl", g_grind_level);
            g_settings.putUChar("roast", (uint8_t)g_roast_type);
            // (no periodic elapsed persistence to avoid flash wear)


            #if SHELLY_ENABLED
            if (!shelly_should_be_on) {
                shelly_should_be_on = true;
                shelly_state_changed = true;
            }
            #endif
            break;
            
        case STATE_IDLE:
            g_preheat_ready_since_ms = 0;
            pump_manual = 0.0f;

            // Clear in-progress marker on any return to idle
            g_settings.putBool("shot_inprog", false);

            // Flush any pending learned state (debounced elsewhere; this is a safe fallback)
            g_grind_advisor.flush();
            if (g_stop_early_dirty && (g_stop_early_pending >= 3 ||
                                      (g_stop_early_last_save_ms != 0 && (now_ms - g_stop_early_last_save_ms) > 600000UL))) {
                g_profiles.save(active_profile_id, active_profile);
                g_stop_early_dirty = false;
                g_stop_early_pending = 0;
                g_stop_early_last_save_ms = now_ms;
                Diag::logf(LOG_INFO, "Auto-save stop_early %.2fg (profile %u)",
                           active_profile.stop_early_g, (unsigned)active_profile_id);
            }
            
            // Switch to brew PID setpoint
            brew_pid.setSetpoint(brew_setpoint_c);
            
            // Shelly stays ON in IDLE so the boiler can hold the selected temperature.
            // (If you want "auto power-off", restore the old OFF-in-IDLE block.)
#if SHELLY_ENABLED
            // no automatic OFF in IDLE
#endif
            break;
            
        case STATE_BREW_PREHEAT:
    pump_manual = 0.0f;
    g_preheat_ready_since_ms = 0;
    brew_pid.setSetpoint(brew_setpoint_c);

    #if SHELLY_ENABLED
    if (!shelly_should_be_on) {
        shelly_should_be_on = true;
        shelly_state_changed = true;
        Serial.println("[SHELLY] ON (BREW)");
    }
    #endif
    break;
            
        case STATE_STEAM_WARMUP:
    #if SHELLY_ENABLED
    if (!shelly_should_be_on) {
        shelly_should_be_on = true;
        shelly_state_changed = true;
        Serial.println("[SHELLY] ON (STEAM)");
    }
    #endif
    // Reset and prepare steam PID
            steam_pid.reset();
            // Warmup boost improves time-to-steam without changing the user's setting.
            g_steam_boost_until_ms = now_ms + 20000; // 20s
            steam_pid.setSetpoint(steam_setpoint_c + 2.0f);
            dreamsteam.reset();
            break;
            
        case STATE_STEAM_READY:
            // DreamSteam is now active
            break;
            case STATE_FLUSH:
            case STATE_DESCALE:
                #if SHELLY_ENABLED
                if (!shelly_should_be_on) {
                    shelly_should_be_on = true;
                    shelly_state_changed = true;
                    Serial.println("[SHELLY] ON (MAINT)");
                }
                #endif
                break;

            case STATE_FAULT:
                // In a fault, cut mains power via Shelly as a hard safety layer.
                #if SHELLY_ENABLED
                if (shelly_should_be_on) {
                    shelly_should_be_on = false;
                    shelly_state_changed = true;
                    Serial.println("[SHELLY] OFF (FAULT)");
                }
                #endif
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
	        esp_task_wdt_reset();
	    // Commands can arrive between control-loop ticks. Use a local timestamp for
	    // state transitions / fault recovery timers.
	    const uint32_t now = millis();
    
    switch(cmd.type) {
        case UserCommand::CMD_STOP: {
            // In FAULT, STOP acts as "ACK" (one-button wife-mode friendly).
            if (current_state == STATE_FAULT) {
                // If we were waiting to finalize a faulted shot, do it now.
                if (g_fault_finalize_pending) {
                    finalize_shot(now, STOP_REASON_FAULT, g_last_weight_g);
                    g_fault_finalize_pending = false;
                    g_fault_drain_until_ms = 0;
                }

                SafetyConfig cfg;
                cfg.max_temp_c = SAFETY_MAX_TEMP_C;
                SafetyInputs si{g_last_temp_c, g_last_weight_g, g_last_zc_ok};

                // Require a short stable window before clearing any fault via STOP.
                // Uses g_stop_ack_since_ms (NOT g_recovery_ok_since_ms which auto-recovery
                // resets every iteration for latched faults, making CMD_STOP permanently broken).
                if (FaultMgr::can_clear_on_ack(current_fault, si, cfg)) {
                    if (g_stop_ack_since_ms == 0) g_stop_ack_since_ms = now;
                    if ((now - g_stop_ack_since_ms) > 2000) {
                        current_fault = FAULT_NONE;
                        g_stop_ack_since_ms = 0;
                        enter_state(STATE_IDLE, now);
                    }
                } else {
                    g_stop_ack_since_ms = 0;
                }
                break;
            }

            // Normal STOP behavior:
            HAL_SetOutputs(0, false, 0);
            g_last_stop_reason = STOP_REASON_USER_STOP;

            // If brewing, go to DRAIN to vent safely and keep the drip-down in the final weight.
            if (current_state == STATE_BREW_DRAIN) {
                enter_state(STATE_IDLE, now);  // finalize immediately via exit action
            } else if (current_state >= STATE_BREW_PREHEAT && current_state <= STATE_BREW_FLOW) {
                enter_state(STATE_BREW_DRAIN, now);
            } else {
                // Not brewing: just return to idle and clear marker
                g_settings.putBool("shot_inprog", false);
                enter_state(STATE_IDLE, now);
            }

            current_fault = FAULT_NONE;
            pump_manual = 0.0f;
            g_manual_pump_warning_sent = false;
            break;
        }

        case UserCommand::CMD_ACK_FAULT:
            if (current_state == STATE_FAULT) {
                SafetyConfig cfg;
                cfg.max_temp_c = SAFETY_MAX_TEMP_C;
                SafetyInputs si{g_last_temp_c, g_last_weight_g, g_last_zc_ok};

                if (FaultMgr::can_clear_on_ack(current_fault, si, cfg)) {
                    current_fault = FAULT_NONE;
                    enter_state(STATE_IDLE, now);
                }
            }
            break;

        case UserCommand::CMD_START_BREW:
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
                // --- Fix 6: If Shelly is OFF, turn it ON and force PREHEAT ---
                // The Shelly relay needs ~1.5s to close and for AC + ZC to stabilize.
                // Without this, safety_check(idle_mode=false) sees ZC_MISSING and blocks.
	                #if SHELLY_ENABLED
	                // Always (re)assert Shelly ON at brew start.
	                // This keeps brew reliable even when Shelly starts OFF after a power cycle.
	                const bool shelly_just_turned_on = !shelly_should_be_on;
	                shelly_should_be_on = true;
	                shelly_state_changed = true; // force resend
	                if (shelly_just_turned_on) {
	                    Serial.println("[BREW] Shelly was OFF — turning ON, forcing PREHEAT for stabilization");
	                } else {
	                    Serial.println("[BREW] Ensuring Shelly ON");
	                }
	                #else
	                const bool shelly_just_turned_on = false;
	                #endif

                // --- Fix 5+6: Skip ZC pre-check if Shelly just turned on ---
                // ZC won't be available yet; the PREHEAT state will wait for it.
                if (!shelly_just_turned_on) {
                    // Strict safety check (idle_mode=false: require TC + ZC healthy)
                    SafetyConfig cfg;
                    cfg.max_temp_c = SAFETY_MAX_TEMP_C;
                    SafetyInputs si{g_last_temp_c, g_last_weight_g, g_last_zc_ok};
                    FaultCode pre_check = safety_check(si, cfg, false);
                    if (pre_check != FAULT_NONE) {
                        Serial.printf("[BREW] Cannot start: safety pre-check failed (fault=%d)\n", (int)pre_check);
                        Serial.printf("[BREW]   temp=%.1f  weight=%.1f  zc_ok=%d\n", g_last_temp_c, g_last_weight_g, g_last_zc_ok ? 1 : 0);
                        break;
                    }
                }

                pump_manual = 0.0f;
                
                #if ENABLE_AUTO_TARE
                HAL_TareScale();
                // --- Fix 1: Reset cached weight AFTER tare to prevent race condition ---
                // Without this, enter_state(PREINFUSE) captures the pre-tare weight
                // into shot_weight_start_g, producing negative yield calculations.
                g_last_weight_g = 0.0f;
                // UI stabilizer baseline should follow tare (display should snap to ~0g)
                g_ui_weight.reset(0.0f);
                // Reset filters/estimators so the new shot starts from a clean baseline
                #if ENABLE_SENSOR_FILTERING
                weight_filter.reset();
                #endif
                g_prev_weight_ms = 0;
                g_weight_rate_ema_g_s = 0.0f;
                g_saved_scale_offset = HAL_GetScaleOffset();
                g_settings.putLong("scale_off", g_saved_scale_offset);
                vTaskDelay(pdMS_TO_TICKS(100));
                #endif
                
                // --- Fix 6: Force PREHEAT if Shelly just turned on ---
                // PREHEAT will wait for temp AND give Shelly relay time to stabilize.
                if (shelly_just_turned_on || g_last_temp_c < (brew_setpoint_c - PREHEAT_TEMP_OFFSET_C)) {
                    // Record the Shelly-on time so PREHEAT can enforce a minimum wait
                    if (shelly_just_turned_on) {
                        g_shelly_on_at_ms = now;
                    }
                    enter_state(STATE_BREW_PREHEAT, now);
                } else {
                    enter_state(STATE_BREW_PREINFUSE, now);
                }
            }
            break;

        case UserCommand::CMD_ENTER_STEAM:
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
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
            // Tare only when safe (no active brewing/steam/flush) to avoid corrupting shot data.
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
                HAL_TareScale();
                g_ui_weight.reset(0.0f);
                g_saved_scale_offset = HAL_GetScaleOffset();
                g_settings.putLong("scale_off", g_saved_scale_offset);
            }
            break;

        case UserCommand::CMD_SET_BREW_TEMP:
            brew_setpoint_c = constrain(cmd.param_f, 85.0f, 105.0f);
            g_settings.putFloat("brew_sp", brew_setpoint_c);
            if (current_state == STATE_IDLE || current_state == STATE_BREW_PREHEAT) {
                brew_pid.setSetpoint(brew_setpoint_c);
            }
            break;

        case UserCommand::CMD_SET_STEAM_TEMP:
            steam_setpoint_c = constrain(cmd.param_f, 130.0f, 155.0f);
            g_settings.putFloat("steam_sp", steam_setpoint_c);
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
                    g_profiles.setActive(active_profile_id);
                    g_settings.putUChar("active_prof", (uint8_t)active_profile_id);
                    refresh_grind_suggestion();
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
            g_settings.putUChar("ds_mode", (uint8_t)config.mode);
            Serial.printf("DreamSteam mode: %d\n", cmd.param_u32);
            break;
        }

        case UserCommand::CMD_TOGGLE_DREAMSTEAM: {
            auto config = dreamsteam.getConfig();
            if (config.mode == DreamSteamController::MODE_DISABLED) {
                const uint8_t saved = (uint8_t)g_settings.getUChar("ds_mode", (uint8_t)DREAMSTEAM_DEFAULT_MODE);
                config.mode = (DreamSteamController::Mode)((saved == 0) ? 1 : saved);
            } else {
                config.mode = DreamSteamController::MODE_DISABLED;
            }
            dreamsteam.setConfig(config);
            g_settings.putUChar("ds_mode", (uint8_t)config.mode);
            break;
        }

        case UserCommand::CMD_CLEAR_SHOT_HISTORY:
            #if ENABLE_SHOT_PERSISTENCE
            shot_history.clear();
            #endif
            g_last_shot_valid = false;
            break;

        case UserCommand::CMD_SET_SCALE_CALIBRATION:
            g_saved_scale_factor = cmd.param_f;
            HAL_SetScaleCalibration(g_saved_scale_factor);
            g_settings.putFloat("scale_fac", g_saved_scale_factor);
            break;

        case UserCommand::CMD_TEST_PUMP:
            // Maintenance: run pump briefly (requires ZC). We keep heater OFF.
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
                const float p = constrain(cmd.param_f, 0.0f, 1.0f);
                const uint32_t dur = constrain((int)cmd.param_u32, 100, 10000); // 0.1..10s
                g_test_pump_power = p;
                g_test_pump_until_ms = now + dur;
                // Open solenoid during pump test to avoid dead-head pressure.
                g_test_solenoid_until_ms = max(g_test_solenoid_until_ms, now + dur);

                #if SHELLY_ENABLED
	                // Always (re)assert Shelly ON for output tests.
	                shelly_should_be_on = true;
	                shelly_state_changed = true;
	                Serial.println("[TEST] Ensuring Shelly ON (pump test)");
                #endif

                Serial.printf("[TEST] Pump: %.0f%% for %lums\n", p * 100.0f, (unsigned long)dur);
            }
            break;

        case UserCommand::CMD_TEST_SOLENOID:
            // Maintenance: open solenoid briefly (heater OFF).
            if (current_state == STATE_IDLE && current_fault == FAULT_NONE) {
                const uint32_t dur = constrain((int)cmd.param_u32, 100, 10000);
                g_test_solenoid_until_ms = max(g_test_solenoid_until_ms, now + dur);

                #if SHELLY_ENABLED
	                shelly_should_be_on = true;
	                shelly_state_changed = true;
	                Serial.println("[TEST] Ensuring Shelly ON (valve test)");
                #endif

                Serial.printf("[TEST] Solenoid: OPEN for %lums\n", (unsigned long)dur);
            }
            break;

        case UserCommand::CMD_RESET_CONTROLLERS:
            brew_pid.reset();
            steam_pid.reset();
            dreamsteam.reset();
            Serial.println("Controllers reset");
            break;

        case UserCommand::CMD_SET_GRIND_LEVEL:
            g_grind_level = (uint16_t)constrain(cmd.param_u32, 0u, 400u);
            g_settings.putUShort("grind_lvl", g_grind_level);
            refresh_grind_suggestion();
            break;

        case UserCommand::CMD_SET_ROAST_TYPE:
            g_roast_type = (RoastType)constrain(cmd.param_u32, 0u, 2u);
            g_settings.putUChar("roast", (uint8_t)g_roast_type);
            refresh_grind_suggestion();
            break;

        case UserCommand::CMD_SET_WIFE_MODE:
            g_wife_mode = (cmd.param_u32 != 0);
            g_settings.putBool("wife_mode", g_wife_mode);
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
    // WDT DISABLED FOR DEBUGGING
    // esp_task_wdt_add(NULL);
    (void)pvParameters;
    init_flow_pid_once_();

    Diag::init();

    // Persistent stores
    g_profiles.begin();
    g_profiles.ensureDefaults();
    g_settings.begin("gaggia", false);
    // Initialize maintenance tracker (NVS-backed)
    g_maintenance.begin();

    // Configure shot phase detector cadence
    g_shot_phase.set_step_ms(CORE_TASK_RATE_MS);

    g_grind_advisor.begin();

    // Load persisted UI/user settings
    g_grind_level = (uint16_t)g_settings.getUShort("grind_lvl", 0);
    g_roast_type = (RoastType)g_settings.getUChar("roast", (uint8_t)ROAST_MEDIUM);
    g_wife_mode = g_settings.getBool("wife_mode", false);

    // Load persisted machine settings
    brew_setpoint_c = constrain(g_settings.getFloat("brew_sp", DEFAULT_BREW_TEMP_C), 85.0f, 105.0f);
    steam_setpoint_c = constrain(g_settings.getFloat("steam_sp", DEFAULT_STEAM_TEMP_C), 130.0f, 155.0f);

    // Restore DreamSteam mode
    {
        DreamSteamController::Config cfg = dreamsteam.getConfig();
        const uint8_t saved = (uint8_t)g_settings.getUChar("ds_mode", (uint8_t)DREAMSTEAM_DEFAULT_MODE);
        cfg.mode = (DreamSteamController::Mode)saved;
        dreamsteam.setConfig(cfg);
    }

    // Restore HX711 scale calibration (applied after HAL_Init)
    g_saved_scale_factor = g_settings.getFloat("scale_fac", DEFAULT_SCALE_CALIBRATION);
    g_saved_scale_offset = g_settings.getLong("scale_off", (long)DEFAULT_SCALE_OFFSET);
    
    // Initialize HAL
    if (!HAL_Init()) {
        Diag::fault(FAULT_BOOT_RESET, "HAL init failed");
        current_state = STATE_FAULT;
        current_fault = FAULT_BOOT_RESET;
    } else {
        Diag::log(LOG_INFO, "HAL initialized");

        // Apply persisted scale calibration now that HX711 is initialized
        HAL_SetScaleCalibration(g_saved_scale_factor);
        HAL_SetScaleOffset(g_saved_scale_offset);

        #if ENABLE_HW_WATCHDOG
        watchdog_timer = timerBegin(1, 80, true); // Timer 1, prescaler 80 -> 1MHz
        timerAttachInterrupt(watchdog_timer, &watchdog_isr, true);
        timerAlarmWrite(watchdog_timer, (uint64_t)WATCHDOG_TIMEOUT_MS * 1000ULL, true);
        timerAlarmEnable(watchdog_timer);
        watchdog_kicked = true;
        Diag::log(LOG_INFO, "Hardware watchdog enabled");
        #endif

        // Brownout / watchdog / panic resets: force safe state and require user ACK.
        const uint32_t rr = HAL_GetResetReason();
	        if (FaultMgr::reset_reason_requires_ack(rr)) {
            current_fault = FAULT_BOOT_RESET;
            Diag::fault(current_fault, "Reset reason %lu requires manual ACK", (unsigned long)rr);
            enter_state(STATE_FAULT, millis());
            HAL_AllOutputsOff();
        } else {
            enter_state(STATE_IDLE, millis());
        }

	        // --- Shelly boot sync ---
	        // Force one initial Shelly command so the relay state is synchronized.
	        // Without this, shelly_should_be_on defaults true and no command is ever sent.
	        #if SHELLY_ENABLED
	        shelly_should_be_on = true;
	        shelly_state_changed = true;
	        g_shelly_on_at_ms = millis();
	        Diag::log(LOG_INFO, "Shelly: initial ON requested");
	        #endif
    }

    // Load active profile (persisted in ProfileStore)
    active_profile_id = g_profiles.getActive();
    if (!g_profiles.load(active_profile_id, active_profile)) {
        Diag::log(LOG_WARN, "Active profile missing, falling back to profile 0");
        active_profile_id = 0;
        g_profiles.setActive(active_profile_id);
        (void)g_profiles.load(active_profile_id, active_profile);
    }

    // Initial grind suggestion for current profile × roast
    refresh_grind_suggestion();

    #if ENABLE_SHOT_PERSISTENCE
    // Initialize shot history from NVS
    if (!shot_history.begin()) {
        Serial.println("WARNING: Shot history initialization failed");
    }
    #endif

    // Reboot-safe interrupted-shot handling
    // If we rebooted mid-shot, record it and clear the flag. Never auto-resume.
    {
        const bool inprog = g_settings.getBool("shot_inprog", false);
        if (inprog) {
            ShotHistory interrupted{};
            interrupted.timestamp = millis() / 1000;
            interrupted.profile_id = (uint8_t)g_settings.getUChar("shot_prof", 0);

            // Elapsed time cannot be recovered safely without frequent flash writes.
            // We log as INTERRUPTED with total_time_ms=0 to avoid flash wear.
            interrupted.total_time_ms = 0;
            interrupted.stop_reason = STOP_REASON_INTERRUPTED;
            interrupted.grind_level = (uint16_t)g_settings.getUShort("grind_lvl", 0);
            interrupted.roast_type = (RoastType)g_settings.getUChar("roast", (uint8_t)ROAST_MEDIUM);

            #if ENABLE_SHOT_PERSISTENCE
            shot_history.saveShot(interrupted);
            #endif

            // Update last-shot UI summary so the user sees what happened after reboot
            g_shot_seq++;
            g_last_shot_valid = true;
            g_last_shot_profile_id = interrupted.profile_id;
            g_last_stop_reason = STOP_REASON_INTERRUPTED;
            g_last_shot_target_yield_g = 0.0f;
            g_last_shot_yield_g = 0.0f;
            g_last_shot_overshoot_g = 0.0f;
            g_last_shot_time_ms = 0;
            g_last_shot_grind_level = interrupted.grind_level;
            g_last_shot_roast_type = interrupted.roast_type;

            g_settings.putBool("shot_inprog", false);
            Diag::log(LOG_WARN, "Interrupted shot recorded (reboot during brew)");
        }
    }


    // Main control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CORE_TASK_RATE_MS);

    while (1) {

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        const uint32_t now = millis();
        esp_task_wdt_reset();

        // Commit maintenance counters (debounced to protect flash)
        g_maintenance.periodic_commit(now);

	        // === READ SENSORS ===
	        float temp, weight;
        bool zc_ok_raw;
        HAL_ReadSensors(temp, weight, zc_ok_raw);
	        const float temp_raw = temp;
	        const float weight_raw = weight;

        // Debounce ZC: some detectors output low-amplitude/noisy pulses that can drop edges.
        // We treat ZC as OK if we have seen at least one good edge within the grace window.
        static uint32_t last_zc_seen_ms = 0;
        if (zc_ok_raw) last_zc_seen_ms = now;
        const bool zc_ok = (last_zc_seen_ms != 0) && ((now - last_zc_seen_ms) <= ZC_MISS_GRACE_MS);

        
	        // Apply filtering to reduce noise
	        #if ENABLE_SENSOR_FILTERING
	        temp = temp_filter.update(temp);
	        weight = weight_filter.update(weight);
	        #endif

	        // Robust spike rejection + EMA smoothing.
	        // Why: pump/dimmer EMI + vibration can inject short spikes that a moving average can't remove.
	        {
	            static float temp_ema = NAN;
	            static float weight_ema = NAN;
	            static float weight_hold = NAN;
	            // Reset weight smoothing when a new shot/tare resets the estimators.
	            if (g_prev_weight_ms == 0) {
	                weight_ema = NAN;
	                weight_hold = NAN;
	            }

	            const bool pump_expected =
	                (g_test_pump_until_ms != 0 && now < g_test_pump_until_ms) ||
	                (current_state == STATE_BREW_PREINFUSE) ||
	                (current_state == STATE_BREW_HOLD) ||
	                (current_state == STATE_BREW_FLOW) ||
	                (current_state == STATE_BREW_DRAIN) ||
	                (current_state == STATE_FLUSH) ||
	                (current_state == STATE_DESCALE);

	            // Temperature: reject impossible jumps (MAX31855 spikes) then EMA.
	            if (isfinite(temp) && temp > -20.0f && temp < 220.0f) {
	                if (isfinite(temp_ema)) {
	                    const float jump = fabsf(temp - temp_ema);
	                    if (jump > 15.0f) {
	                        // treat as glitch
	                        temp = temp_ema;
	                    }
	                }
	                const float alpha_t = pump_expected ? 0.22f : 0.10f;
	                if (!isfinite(temp_ema)) temp_ema = temp;
	                temp_ema += alpha_t * (temp - temp_ema);
	                temp = temp_ema;
	            } else if (isfinite(temp_ema)) {
	                // keep last known good
	                temp = temp_ema;
	            }

	            // Weight: limit per-sample step during pump phases (vibration/EMI), then EMA.
	            if (isfinite(weight) && weight > -5000.0f && weight < 5000.0f) {
	                if (isfinite(weight_ema) && pump_expected) {
	                    // 2.0g per 50ms tick shows up as visible "2g wiggles".
	                    // Typical espresso flow is ~0.05–0.20g per tick; this is still generous.
	                    const float max_step_g = 0.4f; // per control tick
	                    const float delta = weight - weight_ema;
	                    if (fabsf(delta) > max_step_g) {
	                        weight = weight_ema + copysignf(max_step_g, delta);
	                    }
	                }
	                const float alpha_w = pump_expected ? 0.22f : 0.08f;
	                if (!isfinite(weight_ema)) weight_ema = weight;
	                weight_ema += alpha_w * (weight - weight_ema);
	
	                // Display stability: in idle (no pump expected), apply a small deadband so digits don't dance.
	                if (!pump_expected) {
	                    if (!isfinite(weight_hold)) weight_hold = weight_ema;
	                    if (fabsf(weight_ema - weight_hold) < 0.15f) {
	                        weight = weight_hold;
	                    } else {
	                        weight_hold = weight_ema;
	                        weight = weight_ema;
	                    }
	                } else {
	                    weight = weight_ema;
	                }
	            } else if (isfinite(weight_ema)) {
	                weight = weight_ema;
	            }

	            // Optional debug (rare): show raw vs filtered when pump expected.
	            static uint32_t last_filt_dbg_ms = 0;
	            if (pump_expected && (now - last_filt_dbg_ms) >= 2000) {
	                last_filt_dbg_ms = now;
	                Serial.printf("[FILT] temp_raw=%.1f temp=%.1f | w_raw=%.2f w=%.2f\n", temp_raw, temp, weight_raw, weight);
	            }
	        }
        
        // Cache for command gating
        g_last_temp_c = temp;
        g_last_weight_g = weight;
        g_last_zc_ok = zc_ok;

        // -----------------------------
        // PREDICTIVE TEMPERATURE (simple rate lookahead)
        // -----------------------------
        // The heater + boiler have inertia, and noisy thermocouple readings can make the loop feel "reactive".
        // We add a small lookahead based on a low-passed dT/dt so PID reacts earlier and swings less.
        float temp_for_pid = temp;
        {
            static float s_prev_temp = NAN;
            static uint32_t s_prev_ms = 0;
            static float s_rate_ema = 0.0f;

            if (isfinite(s_prev_temp) && s_prev_ms != 0) {
                const float dt_s = (now > s_prev_ms) ? ((now - s_prev_ms) / 1000.0f) : 0.0f;
                if (dt_s > 0.01f) {
                    float rate = (temp - s_prev_temp) / dt_s;        // C/s
                    rate = constrain(rate, -5.0f, 5.0f);             // reject spikes
                    s_rate_ema = (0.85f * s_rate_ema) + (0.15f * rate);
                }
            }
            s_prev_temp = temp;
            s_prev_ms = now;

            const float lookahead_s = (current_state == STATE_STEAM_WARMUP || current_state == STATE_STEAM_READY) ? 0.9f : 1.2f;
            temp_for_pid = temp + (s_rate_ema * lookahead_s);
        }

        // === PERIODIC SENSOR DIAGNOSTICS (every 2 seconds) ===
        {
            static uint32_t last_diag_ms = 0;
            if ((now - last_diag_ms) >= 2000) {
                last_diag_ms = now;
                const char* state_name = "?";
                switch(current_state) {
                    case STATE_BOOT: state_name = "BOOT"; break;
                    case STATE_IDLE: state_name = "IDLE"; break;
                    case STATE_FAULT: state_name = "FAULT"; break;
                    case STATE_BREW_PREHEAT: state_name = "PREHEAT"; break;
                    case STATE_BREW_PREINFUSE: state_name = "PREINFUSE"; break;
                    case STATE_BREW_HOLD: state_name = "HOLD"; break;
                    case STATE_BREW_FLOW: state_name = "FLOW"; break;
                    case STATE_BREW_DRAIN: state_name = "DRAIN"; break;
                    case STATE_STEAM_WARMUP: state_name = "STEAM_WARM"; break;
                    case STATE_STEAM_READY: state_name = "STEAM_RDY"; break;
                    case STATE_FLUSH: state_name = "FLUSH"; break;
                    case STATE_DESCALE: state_name = "DESCALE"; break;
                    default: break;
                }
                Serial.printf("[DIAG] state=%s fault=%d | temp=%.1f°C weight=%.1fg zc=%s | setpoint=%.1f | shelly=%s\n",
                    state_name, (int)current_fault,
                    temp, weight, zc_ok ? "OK" : "MISSING",
                    brew_setpoint_c,
                    shelly_should_be_on ? "ON" : "OFF");
                
                // Extra detail when sensors look bad
                if (temp > 900.0f || isnan(temp)) {
                    Serial.println("[DIAG] *** THERMOCOUPLE: NaN or >900°C — check MAX31855 wiring (CS=42 DO=47 CLK=48)");
                }
                if (!zc_ok) {
                    Serial.println("[DIAG] *** ZERO CROSS: no signal — check ZC detector on GPIO39");
                }
                if (weight <= -900.0f) {
                    Serial.println("[DIAG] *** SCALE: timeout — check HX711 wiring (DT=17 SCK=18)");
                }
            }
        }

        // Update weight-rate estimator during active shot (used for flow display + predictive weight stop)
        const bool weight_ok = isfinite(weight) && (weight > -900.0f);
        const bool shot_active = (current_state >= STATE_BREW_PREINFUSE && current_state <= STATE_BREW_DRAIN);
        if (weight_ok) {
            // Update flow estimator at the control rate (20 Hz).
	            // Feed FlowMeter the raw scale signal so it can reject spikes without the "max_step" clamp.
	            g_flow_meter.update(now, weight_raw, shot_active);
            g_weight_rate_ema_g_s = g_flow_meter.state().flow_ema_gps;

            // Extraction-phase detector (starts timer when flow actually begins).
            g_shot_phase.update(now, shot_active, g_flow_meter.state().flow_valid, g_flow_meter.state().flow_mlps);
        }
#if ENABLE_TEMP_PREDICTION
        temp_predictor.update(temp, now);
        #endif

        // === AUTO-RECOVERY (recoverable faults only) ===
        if (current_state == STATE_FAULT && current_fault != FAULT_NONE && is_recoverable_fault(current_fault)) {
            SafetyConfig cfg;
            cfg.max_temp_c = SAFETY_MAX_TEMP_C;
            SafetyInputs si{temp, weight, zc_ok};
            const bool conditions_ok = (safety_check(si, cfg, false) == FAULT_NONE);
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

        // === PROCESS UI COMMANDS ===
        // MUST run before fault detection so the user can ALWAYS clear faults
        // via CMD_STOP / CMD_ACK_FAULT, even during persistent sensor failures.
        process_commands();

        // === FAULT DETECTION (standardized) ===
        // Skip if already in STATE_FAULT — auto-recovery and user ACK handle the exit.
        // Re-running fault detection while in FAULT would (a) call enter_state(STATE_FAULT)
        // again, destroying drain/finalize timers, and (b) was previously blocking command
        // processing via `continue`.
        if (current_state != STATE_FAULT) {
            const SystemState state_before_fault = current_state;

            SafetyConfig cfg;
            cfg.max_temp_c = SAFETY_MAX_TEMP_C;
            SafetyInputs si{temp, weight, zc_ok};

            // In IDLE/PREHEAT, use relaxed safety (only OVERTEMP is fatal).
            // During active brew/steam, enforce full safety (TC, ZC, weight).
            const bool idle_mode = (current_state == STATE_IDLE || 
                                    current_state == STATE_BREW_PREHEAT ||
                                    current_state == STATE_BOOT);
            FaultCode f = safety_check(si, cfg, idle_mode);
            if (f != FAULT_NONE) {
                const bool was_brewing = (state_before_fault >= STATE_BREW_PREHEAT && state_before_fault <= STATE_BREW_DRAIN);
                const bool shot_started = (shot_start_ms != 0);

                // Enter fault state
                enter_state(STATE_FAULT, now);
                current_fault = f;
                Diag::fault(f, "Safety fault triggered");

                // If a shot was in progress, vent briefly and finalize after vent window
                if (was_brewing && shot_started) {
                    g_last_stop_reason = STOP_REASON_FAULT;
                    g_fault_finalize_pending = true;
                    g_fault_finalize_at_ms = now + DRAIN_TIME_MS;
                    g_fault_drain_until_ms = g_fault_finalize_at_ms;

                    // Clear reboot marker now (fault is not an "interrupted shot")
                    g_settings.putBool("shot_inprog", false);

                    // Immediate pressure relief (heater/pump off, solenoid open)
                    HAL_SetOutputs(0, true, 0);
                } else {
                    HAL_SetOutputs(0, false, 0);
                }

                StateSnapshot snap{};
                snap.state = STATE_FAULT;
                snap.fault = f;
                snap.temp_c = temp;
                snap.weight_g = weight;
                snap.timestamp = now;
                snap.wifi_connected = WiFi.isConnected();
                snap.wifi_rssi = snap.wifi_connected ? WiFi.RSSI() : 0;

                // Keep last-shot fields so the UI can show context
                snap.last_shot_valid = g_last_shot_valid;
                snap.last_shot_profile_id = g_last_shot_profile_id;
                snap.last_shot_stop_reason = g_last_stop_reason;
                snap.last_shot_target_yield_g = g_last_shot_target_yield_g;
                snap.last_shot_yield_g = g_last_shot_yield_g;
                snap.last_shot_overshoot_g = g_last_shot_overshoot_g;
                snap.last_shot_time_ms = g_last_shot_time_ms;
                snap.last_shot_grind_level = g_last_shot_grind_level;
                snap.last_shot_roast_type = g_last_shot_roast_type;
                snap.last_shot_seq = g_shot_seq;

                // Advisor output stays visible even in fault
                snap.grind_level = g_grind_level;
                snap.roast_type = g_roast_type;
                snap.wife_mode = g_wife_mode;
                snap.suggested_grind_level = g_suggested_grind;
                snap.suggested_grind_confidence = g_suggested_conf;
                snap.suggested_grind_delta = g_suggested_delta;

                xQueueOverwrite(queue_to_ui, &snap);
                // NOTE: no `continue` here — fall through to STATE_FAULT handler
                // in the state machine, which safely keeps all outputs off.
            }
        }

        // === STATE MACHINE ===
        uint8_t pump_percent = 0;
        bool solenoid_open = false;
        uint8_t heater_duty = 0;
        float pid_output = 0.0f;
        
        // PID diagnostics
        float pid_p = 0.0f, pid_i = 0.0f, pid_d = 0.0f;

        if (current_state == STATE_FAULT) {
            // Everything off in fault state.
            heater_duty = 0;
            pump_percent = 0;

            // If a fault happened mid-shot, vent briefly to relieve pressure.
            solenoid_open = (g_fault_drain_until_ms != 0 && now < g_fault_drain_until_ms);

            // Finalize an interrupted-by-fault shot after the vent window (captures drip-down).
            if (g_fault_finalize_pending && now >= g_fault_finalize_at_ms) {
                finalize_shot(now, STOP_REASON_FAULT, g_last_weight_g);
                g_fault_finalize_pending = false;
                g_fault_drain_until_ms = 0;
            }
        }
        else if (current_state == STATE_IDLE) {
            // Brew PID temperature control (only if thermocouple is valid)
            const bool temp_valid = !isnan(temp) && temp < 900.0f;

            // Shelly does not gate heater PID in this build (SSR assumed independent of Shelly).
            if (temp_valid) {
                pid_output = brew_pid.update(temp_for_pid, now);
                heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);

                // --- Fix 8: Soft landing / glide path ---
                // The Gaggia's small boiler has significant thermal inertia.
                // Full PID output near setpoint causes massive overshoot.
                const float error = brew_setpoint_c - temp;

                if (error < 0.0f) {
                    // Already above setpoint → heater OFF immediately (anti-windup)
                    heater_duty = 0;
                    pid_output = 0.0f;
                    // Force integral to zero to prevent further windup
                    brew_pid.reset();
                    brew_pid.setSetpoint(brew_setpoint_c);
                } else if (error < BREW_CLOSE_RANGE_C) {
                    // Very close to setpoint → cap power
                    heater_duty = min(heater_duty, (uint8_t)BREW_CLOSE_MAX_DUTY);
                } else if (error < BREW_GLIDE_RANGE_C) {
                    // Approaching setpoint → proportional cap
                    heater_duty = min(heater_duty, (uint8_t)BREW_GLIDE_MAX_DUTY);
                }

                // --- Fix 8: Hard safety limit in brew mode ---
                if (temp >= BREW_HARD_TEMP_LIMIT_C) {
                    heater_duty = 0;
                    pid_output = 0.0f;
                    Serial.printf("[SAFETY] Hard brew temp limit %.0f°C hit (temp=%.1f°C)\n",
                        BREW_HARD_TEMP_LIMIT_C, temp);
                }
            } else {
                // Thermocouple not reading — heater stays off for safety
                pid_output = 0.0f;
                heater_duty = 0;
            }
            
            #if ENABLE_PID_DIAGNOSTICS
            pid_p = brew_pid.lastPTerm();
            pid_i = brew_pid.lastITerm();
            pid_d = brew_pid.lastDTerm();
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
            // Only run PID if temp is valid (NaN guard)
            const bool temp_valid_ph = !isnan(temp) && temp < 900.0f;
            if (temp_valid_ph) {
                pid_output = brew_pid.update(temp_for_pid, now);
                heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            } else {
                pid_output = 0.0f;
                heater_duty = 0;
            }
            pump_percent = 0;
            solenoid_open = false;

            // --- Fix 6: Enforce Shelly stabilization delay ---
            // If Shelly was just turned on for this brew, wait for relay + ZC to stabilize.
            const bool shelly_stable = (g_shelly_on_at_ms == 0) ||
                                       ((now - g_shelly_on_at_ms) >= SHELLY_STABILIZE_MS);

            // Require: temp near setpoint and Shelly stable
            if (temp_valid_ph && temp >= (brew_setpoint_c - 1.0f) && shelly_stable) {
                if (g_preheat_ready_since_ms == 0) g_preheat_ready_since_ms = now;
                if ((now - g_preheat_ready_since_ms) >= 1500) {
                    g_shelly_on_at_ms = 0; // consumed
                    enter_state(STATE_BREW_PREINFUSE, now);
                }
            } else {
                g_preheat_ready_since_ms = 0;
            }
        }
        else if (current_state == STATE_BREW_PREINFUSE) {
            // Slayer-style PRE-BREW (restricted flow).
            // We emulate the Slayer needle-valve "pre-brew" by running the pump at reduced power
            // until the puck is saturated, then switching to Full-Brew.
            brew_pid.setSetpoint(active_profile.brew_temp_c);
            pid_output = brew_pid.update(temp_for_pid, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            solenoid_open = true;
            pump_percent = (uint8_t)(active_profile.preinfuse_pump * 100.0f);

            // Collect statistics
            shot_temp_sum += temp;
            shot_temp_min = min(shot_temp_min, temp);
            shot_temp_max = max(shot_temp_max, temp);
            shot_temp_samples++;

            const uint32_t phase_elapsed = now - phase_start_ms;

            // In a classic "Slayer shot", you stay in pre-brew until the puck is fully saturated.
            // Slayer's own guidance mentions extracting a small amount (2–5g) in pre-brew before going full-brew.
            static constexpr float SLAYER_PREBREW_EXTRACT_G = 4.0f;
            float yield_g = 0.0f;
            if (weight_ok && weight_valid(shot_weight_start_g)) {
                yield_g = weight - shot_weight_start_g;
            }

            // Advance to Full-Brew once we've extracted a few grams, or once the pre-brew time cap is reached.
            if ((weight_ok && yield_g >= SLAYER_PREBREW_EXTRACT_G && phase_elapsed >= 1500) ||
                (phase_elapsed >= active_profile.preinfuse_ms)) {
                if (active_profile.bloom_ms > 0) {
                    enter_state(STATE_BREW_HOLD, now);
                } else {
                    enter_state(STATE_BREW_FLOW, now);
                }
            }
        }
        else if (current_state == STATE_BREW_HOLD) {
            brew_pid.setSetpoint(active_profile.brew_temp_c);
            pid_output = brew_pid.update(temp_for_pid, now);
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
            // Temperature compensation during flow: give a little headroom for thermal load
            brew_pid.setSetpoint(active_profile.brew_temp_c + active_profile.flow_temp_offset_c);
            pid_output = brew_pid.update(temp_for_pid, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            solenoid_open = true;

            const uint32_t elapsed = now - phase_start_ms;
            const uint32_t remaining_time = (elapsed >= active_profile.brew_ms) ? 0 : (active_profile.brew_ms - elapsed);

            // Predictive weight stop
            const float yield_g = weight - shot_weight_start_g;
            bool should_stop_by_weight = false;
            uint32_t remaining_weight_ms = 0xFFFFFFFFu;
            if (weight_ok && weight_valid(shot_weight_start_g) && (elapsed >= 3000) && active_profile.target_yield_g > 5.0f) {
                const float rate = max(g_weight_rate_ema_g_s, 0.10f); // g/s
                const float lag_g = rate * (active_profile.stop_lag_ms / 1000.0f);
                const float early_g = max(active_profile.stop_early_g, lag_g);
                // --- Fix 2: Clamp cutoff to prevent negative/zero ---
                // If stop_early_g > target_yield_g (misconfigured profile),
                // cutoff would be negative → any positive yield triggers instant stop.
                const float cutoff_g = max(0.1f, active_profile.target_yield_g - early_g);

                if (yield_g >= cutoff_g) {
                    should_stop_by_weight = true;
                } else {
                    const float rem_g = cutoff_g - yield_g;
                    remaining_weight_ms = (uint32_t)constrain((rem_g / rate) * 1000.0f, 0.0f, 300000.0f);
                }
            }

            const uint32_t remaining_ms = (remaining_weight_ms < remaining_time) ? remaining_weight_ms : remaining_time;

            // Pump profile (smooth transitions, no discrete jumps)
            // Desired behavior:
            //   Pre-brew:    active_profile.preinfuse_pump (e.g. 60%) in STATE_BREW_PREINFUSE
            //   Full brew:   active_profile.brew_pump      (e.g. 100%)
            //   Decline:     smoothly ramp to active_profile.flow_end_pump (e.g. 60%) for the last SLAYER_DECLINE_LAST_G grams
            static constexpr float SLAYER_DECLINE_LAST_G = 5.0f;

            const float base_full = active_profile.brew_pump * 100.0f;
            const float base_end  = active_profile.flow_end_pump * 100.0f;
            float base_cmd = base_full;

            // Smooth decline based on yield progress (last N grams).
            if (weight_ok && active_profile.target_yield_g > (SLAYER_DECLINE_LAST_G + 0.5f)) {
                const float decline_start_g = max(0.0f, active_profile.target_yield_g - SLAYER_DECLINE_LAST_G);
                const float t = constrain((yield_g - decline_start_g) / SLAYER_DECLINE_LAST_G, 0.0f, 1.0f);
                const float s = t * t * (3.0f - 2.0f * t); // smoothstep
                base_cmd = (1.0f - s) * base_full + s * base_end;
            }

            pump_percent = (uint8_t)constrain((int)(base_cmd + 0.5f), 0, 100);
            // Optional: flow profiling (closed-loop on scale-derived flow).
            // Targets are in ml/s. With density_g_per_ml=1.00, 1 ml/s ≈ 1 g/s.
            if (active_profile.enable_flow_profile) {
                const auto &fs = g_flow_meter.state();

                // Feed-forward base command mapped from target flow.
                // 2.0 ml/s -> 100% pump; 1.2 ml/s -> 60% pump (good starting point for vibe + dimmer).
                float ff_cmd = constrain((active_profile.flow_target_ml_s / 2.0f) * 100.0f, 30.0f, 100.0f);

                // Setpoint (ml/s) with a gentle taper in the last SLAYER_DECLINE_LAST_G grams.
                float sp_mlps = active_profile.flow_target_ml_s;
                if (weight_ok && active_profile.target_yield_g > (SLAYER_DECLINE_LAST_G + 0.5f)) {
                    const float decline_start_g = max(0.0f, active_profile.target_yield_g - SLAYER_DECLINE_LAST_G);
                    const float t = constrain((yield_g - decline_start_g) / SLAYER_DECLINE_LAST_G, 0.0f, 1.0f);
                    const float s = t * t * (3.0f - 2.0f * t); // smoothstep

                    const float end_ratio = constrain(active_profile.flow_end_pump / max(active_profile.brew_pump, 0.01f), 0.50f, 1.0f);
                    sp_mlps = (1.0f - s) * sp_mlps + s * (sp_mlps * end_ratio);
                    ff_cmd = (1.0f - s) * ff_cmd + s * (ff_cmd * end_ratio);
                }

                // Only close the loop once flow is actually detected (prevents windup).
                if (fs.flow_valid) {
                    g_flow_pid.setSetpoint(sp_mlps);
                    const float trim = g_flow_pid.update(fs.flow_mlps, now);
                    const float cmd = constrain(ff_cmd + trim, 0.0f, 100.0f);
                    pump_percent = (uint8_t)cmd;
                } else {
                    // Use feed-forward while waiting for first drops, and keep PID reset.
                    pump_percent = (uint8_t)ff_cmd;
                    g_flow_pid.reset();
                }
            }


            shot_temp_sum += temp;
            shot_temp_min = min(shot_temp_min, temp);
            shot_temp_max = max(shot_temp_max, temp);
            shot_temp_samples++;

            // Stop conditions:
            // - PRIMARY: scale target (36g etc.)
            // - Time is NOT a stop condition when a weight target is enabled (it is only a guide for grinder dial-in).
            // - Safety: hard timeout prevents endless pumping on sensor/flow failures.
            static constexpr uint32_t BREW_SAFETY_MAX_MS = 75000; // 75s hard safety cap
            const bool target_yield_enabled = (active_profile.target_yield_g > 5.0f);
            const bool weight_stop_enabled = target_yield_enabled && weight_ok && weight_valid(shot_weight_start_g);

            if (should_stop_by_weight) {
                g_last_stop_reason = STOP_REASON_WEIGHT_TARGET;
                heater_duty = 0;
                pump_percent = 0;
                solenoid_open = true;
                enter_state(STATE_BREW_DRAIN, now);
            } else if (!target_yield_enabled && (elapsed >= active_profile.brew_ms)) {
                // Only stop by time if weight target is disabled or scale is invalid.
                g_last_stop_reason = STOP_REASON_TIME_CAP;
                heater_duty = 0;
                pump_percent = 0;
                solenoid_open = true;
                enter_state(STATE_BREW_DRAIN, now);
            } else if (elapsed >= BREW_SAFETY_MAX_MS) {
                // Safety timeout (treated as TIME_CAP for UI compatibility)
                g_last_stop_reason = STOP_REASON_TIME_CAP;
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

            // Dynamic setpoint: brief warmup boost, then settle to user setpoint
            if (g_steam_boost_until_ms != 0 && now >= g_steam_boost_until_ms) {
                g_steam_boost_until_ms = 0;
            }
            const float steam_sp = (g_steam_boost_until_ms != 0) ? (steam_setpoint_c + 2.0f) : steam_setpoint_c;
            steam_pid.setSetpoint(steam_sp);

            // Steam PID control
            pid_output = steam_pid.update(temp_for_pid, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);

            if (temp >= steam_setpoint_c - 0.5f) {
                enter_state(STATE_STEAM_READY, now);
            }
        }
        else if (current_state == STATE_STEAM_READY) {
            solenoid_open = false;
            
            // Steam PID control for heater
            pid_output = steam_pid.update(temp_for_pid, now);
            heater_duty = (uint8_t)constrain(pid_output, 0.0f, 100.0f);
            
            // DreamSteam pump assist
            pump_percent = dreamsteam.update(temp, steam_setpoint_c, now, zc_ok);

            // Lightweight debug: confirms whether DreamSteam is actually requesting pump pulses.
            static uint32_t s_last_ds_log_ms = 0;
            if (now - s_last_ds_log_ms > 1000) {
                s_last_ds_log_ms = now;
                Serial.printf("[DREAMSTEAM] enabled=%d pump=%d%% temp=%.1f set=%.1f zc_ok=%d\n",
                              (int)dreamsteam.isEnabled(), pump_percent, (double)temp, (double)steam_setpoint_c, (int)zc_ok);
            }
            
            #if ENABLE_PID_DIAGNOSTICS
            pid_p = steam_pid.lastPTerm();
            pid_i = steam_pid.lastITerm();
            pid_d = steam_pid.lastDTerm();
            #endif
        }
        // === FINAL SAFETY INTERLOCK ===
        // ZC is not a hard requirement for pump operation in this build.
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

        // === FLOW STALL DETECTION (choked shot / scale failure) ===
        #if ENABLE_FLOW_STALL_DETECTION
        {
            const bool in_brew = (current_state >= STATE_BREW_PREINFUSE && current_state <= STATE_BREW_FLOW);
            if (in_brew && pump_percent > 20 && weight_ok && weight_valid(shot_weight_start_g)) {
                const float yield_g = max(0.0f, weight - shot_weight_start_g);

                // Consider "progress" only when yield increases materially (filters noise)
                if ((yield_g - g_flow_last_yield_g) > 0.30f) {
                    g_flow_last_yield_g = yield_g;
                    g_flow_last_progress_ms = now;
                }

                const uint32_t since_start = (shot_start_ms == 0) ? 0 : (now - shot_start_ms);
                if (since_start > 5000 && g_flow_last_progress_ms != 0 && (now - g_flow_last_progress_ms) > FLOW_STALL_TIMEOUT_MS) {
                    // Escalate to fault with a brief vent + delayed finalize to capture drip-down.
                    current_fault = FAULT_FLOW_STALL;
                    g_last_stop_reason = STOP_REASON_FAULT;
                    g_fault_finalize_pending = true;
                    g_fault_finalize_at_ms = now + 2000;
                    g_fault_drain_until_ms = g_fault_finalize_at_ms;
                    enter_state(STATE_FAULT, now);
                    pump_percent = 0;
                    heater_duty = 0;
                    solenoid_open = true;
                    Diag::fault(current_fault, "Flow stalled (choked shot or scale)");
                }
            }
        }
        #endif

        // Temp validity for heuristic fault detectors (NaN/TC-fail → 999°C would cause false positives)
const bool temp_reading_valid = !isnan(temp) && temp < 900.0f;

// Hard overtemp kill: regardless of SSR polarity, cut Shelly and enter FAULT.
if (temp_reading_valid && temp >= SAFETY_MAX_TEMP_C && current_fault == FAULT_NONE) {
    current_fault = FAULT_OVERTEMP;
    enter_state(STATE_FAULT, now);
    pump_percent = 0;
    heater_duty = 0;
    solenoid_open = false;
    Diag::fault(current_fault, "Hard overtemp limit hit");
}

        // === PID SATURATION FAULT (heater not heating) ===
        // Only check with valid temp readings (TC failure → 999°C would cause bogus gap)
        if (temp_reading_valid &&
            (current_state == STATE_IDLE || current_state == STATE_BREW_PREHEAT || current_state == STATE_STEAM_WARMUP) &&
            heater_duty >= 95 && zc_ok) {
            const float sp = (current_state == STATE_STEAM_WARMUP) ? steam_setpoint_c : brew_setpoint_c;
            if ((sp - temp) > 10.0f) {
                if (g_pid_sat_start_ms == 0) {
                    g_pid_sat_start_ms = now;
                    g_pid_sat_start_temp = temp;
                } else if ((now - g_pid_sat_start_ms) > PID_SATURATION_TIMEOUT_MS) {
                    // If temperature barely moved while heater was effectively pegged, flag.
                    if ((temp - g_pid_sat_start_temp) < 2.0f) {
                        current_fault = FAULT_PID_SATURATION;
                        enter_state(STATE_FAULT, now);
                        pump_percent = 0;
                        heater_duty = 0;
                        solenoid_open = false;
                        Diag::fault(current_fault, "Heater saturated but temperature not rising");
                    }
                    g_pid_sat_start_ms = 0;
                }
            } else {
                g_pid_sat_start_ms = 0;
            }
        } else {
            g_pid_sat_start_ms = 0;
        }

        // Heater stuck-ON heuristic
        // Guard: only run when temp readings are valid (TC failure → 999°C would
        // cause a false positive since "rise" would be huge)
        if (current_fault == FAULT_NONE && temp_reading_valid) {
            const bool heater_cmd_off = (heater_duty <= 0);
            if (heater_cmd_off) {
                if (g_heater_off_start_ms == 0) {
                    g_heater_off_start_ms = now;
                    g_heater_off_start_temp = temp;
                } else if ((now - g_heater_off_start_ms) >= 60000) {
                    const float rise = temp - g_heater_off_start_temp;
                    if (rise >= 10.0f && temp < SAFETY_MAX_TEMP_C) {
                        current_fault = FAULT_HEATER_STUCK_ON;
                        enter_state(STATE_FAULT, now);
                        pump_percent = 0;
                        heater_duty = 0;
                        solenoid_open = false;
                        Diag::fault(current_fault, "Temp rising while heater OFF");
                    }
                    g_heater_off_start_ms = 0;
                }
            } else {
                g_heater_off_start_ms = 0;
            }
        } else {
            g_heater_off_start_ms = 0;
        }

        // === OUTPUT TEST OVERRIDE ===
const bool test_active = (g_test_pump_until_ms != 0 && now < g_test_pump_until_ms) ||
                         (g_test_solenoid_until_ms != 0 && now < g_test_solenoid_until_ms);

if (test_active) {
    // Never heat during output tests, and prevent PID windup.
    heater_duty = 0;

    if (current_state == STATE_IDLE) {
        brew_pid.reset();
        brew_pid.setSetpoint(brew_setpoint_c);
    }

    if (g_test_pump_until_ms != 0 && now < g_test_pump_until_ms) {
        pump_percent = (uint8_t)constrain(g_test_pump_power * 100.0f, 0.0f, 100.0f);
        solenoid_open = true;
    } else {
        pump_percent = 0;
    }

    if (g_test_solenoid_until_ms != 0 && now < g_test_solenoid_until_ms) {
        solenoid_open = true;
    }
} else {
    // Clear expired windows
    if (g_test_pump_until_ms != 0 && now >= g_test_pump_until_ms) {
        g_test_pump_until_ms = 0;
        g_test_pump_power = 0.0f;
    }
    if (g_test_solenoid_until_ms != 0 && now >= g_test_solenoid_until_ms) {
        g_test_solenoid_until_ms = 0;
    }

    // In IDLE, keep Shelly ON so the boiler can regulate temperature.
    #if SHELLY_ENABLED
    // no automatic OFF in IDLE
    #endif
}

// Apply outputs
// Pump diagnostics during active brew phases
{
    static uint32_t last_pump_diag_ms = 0;
    const bool in_brew = (current_state >= STATE_BREW_PREHEAT && current_state <= STATE_BREW_DRAIN);
    if (in_brew && (now - last_pump_diag_ms) >= 500) {
        last_pump_diag_ms = now;
        Serial.printf("[PUMP] state=%d pump=%u%% heater=%u%% sol=%d profile_brew_pump=%.0f%%\n",
            (int)current_state, pump_percent, heater_duty, solenoid_open ? 1 : 0,
            active_profile.brew_pump * 100.0f);
    }
}
HAL_SetOutputs(pump_percent, solenoid_open, heater_duty);

        // Send Shelly update if state changed
        #if SHELLY_ENABLED
        // Keep Shelly state resilient: if the Shelly was toggled externally (app/manual)
        // we periodically re-assert our desired state while the machine is running.
        {
            static uint32_t last_shelly_heartbeat_ms = 0;
            const uint32_t HEARTBEAT_MS = 15000;
            if ((now - last_shelly_heartbeat_ms) > HEARTBEAT_MS) {
                last_shelly_heartbeat_ms = now;
                if (shelly_should_be_on) {
                    shelly_state_changed = true;
                }
            }
        }

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
        // Display filtered weight from FlowMeter (median+EMA). Much steadier than raw HX711 units.
        // Further stabilize the UI readout:
        // - During a shot, show a responsive short-window average.
        // - In idle, "lock" to a baseline and only update when a long-window average
        //   truly shifts by >= ~0.5g (ignores vibration/EMI wiggle).
        const bool ui_dynamic_weight =
            (current_state == STATE_BREW_PREINFUSE) ||
            (current_state == STATE_BREW_HOLD) ||
            (current_state == STATE_BREW_FLOW) ||
            (current_state == STATE_BREW_DRAIN) ||
            (current_state == STATE_FLUSH) ||
            (current_state == STATE_DESCALE);
        // UI weight: 1s block averaging + hysteresis (see stable_weight.h)
        // Feed raw scale grams to maximize cancellation of vibration wiggle.
        snap.weight_g = g_ui_weight.update(now, g_flow_meter.state().weight_raw_g, ui_dynamic_weight);
        // In the UI we want the *active* target (brew vs steam), not always the brew target.
        snap.brew_setpoint_c = (current_state == STATE_STEAM) ? steam_setpoint_c : brew_setpoint_c;
        snap.steam_setpoint_c = steam_setpoint_c;
        snap.pump_manual = pump_manual;
        snap.timestamp = now;
        snap.shelly_latched = shelly_should_be_on;
        snap.active_profile_id = active_profile_id;

        // Connectivity
        snap.wifi_connected = WiFi.isConnected();
        snap.wifi_rssi = snap.wifi_connected ? WiFi.RSSI() : 0;

        // Diagnostics helpers
        snap.dreamsteam_mode = (uint8_t)dreamsteam.getConfig().mode;
        snap.scale_factor = HAL_GetScaleCalibration();

        snap.grind_level = g_grind_level;
        snap.roast_type = g_roast_type;
        snap.wife_mode = g_wife_mode;
        snap.suggested_grind_level = g_suggested_grind;
        snap.suggested_grind_confidence = g_suggested_conf;
        snap.suggested_grind_delta = g_suggested_delta;

        snap.last_shot_valid = g_last_shot_valid;
        snap.last_shot_profile_id = g_last_shot_profile_id;
        snap.last_shot_stop_reason = g_last_stop_reason;
        snap.last_shot_target_yield_g = g_last_shot_target_yield_g;
        snap.last_shot_yield_g = g_last_shot_yield_g;
        snap.last_shot_overshoot_g = g_last_shot_overshoot_g;
        snap.last_shot_time_ms = g_last_shot_time_ms;
        snap.last_shot_seq = g_shot_seq;
        snap.last_shot_grind_level = g_last_shot_grind_level;
        snap.last_shot_roast_type = g_last_shot_roast_type;

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

        // Derived extraction metrics (best-effort)
        {
            const auto fm = g_flow_meter.state();
            const bool weight_ok = isfinite(fm.weight_filt_g) && (fm.weight_filt_g > -900.0f);
            const bool shot_active = (current_state == STATE_BREW_PREINFUSE || current_state == STATE_BREW_HOLD || current_state == STATE_BREW_FLOW || current_state == STATE_BREW_DRAIN);
            if (shot_active && weight_ok) {
                // Use FlowMeter's filtered/relative values for stable UI and better stop accuracy.
                snap.yield_g = max(0.0f, fm.total_g);
                snap.target_yield_g = active_profile.target_yield_g;
                snap.weight_rate_g_s = fm.flow_ema_gps;
                snap.flow_rate = snap.weight_rate_g_s;
                snap.flow_std_g_s = fm.flow_std;
                snap.extraction_ms = g_shot_phase.extraction_ms(now);

                snap.shot_total_cap_ms = active_profile.preinfuse_ms + active_profile.bloom_ms + active_profile.brew_ms;

                // ETA to shot end (prefer weight target ETA if it looks stable)
                uint32_t eta_ms = 0;
                if (current_state == STATE_BREW_FLOW && active_profile.target_yield_g > 0.1f && g_weight_rate_ema_g_s > 0.10f) {
                    const uint32_t elapsed = now - phase_start_ms;
                    const float rate = max(g_weight_rate_ema_g_s, 0.10f);
                    const float lag_g = rate * (active_profile.stop_lag_ms / 1000.0f);
                    const float early_g = max(active_profile.stop_early_g, lag_g);
                    const float cutoff_g = active_profile.target_yield_g - early_g;
                    if (snap.yield_g < cutoff_g) {
                        const float rem_g = cutoff_g - snap.yield_g;
                        eta_ms = (uint32_t)constrain((rem_g / rate) * 1000.0f, 0.0f, 300000.0f);
                    }
                }
                if (eta_ms == 0 && snap.shot_total_cap_ms > snap.shot_ms) {
                    eta_ms = snap.shot_total_cap_ms - snap.shot_ms;
                }
                snap.eta_to_shot_end_ms = eta_ms;
            } else {
                snap.yield_g = 0.0f;
                snap.target_yield_g = 0.0f;
                snap.weight_rate_g_s = 0.0f;
                snap.eta_to_shot_end_ms = 0;
                snap.shot_total_cap_ms = 0;
                // Keep snap.flow_rate at 0 by default
            }
        }

        // Temperature prediction (ETA to ready)
        #if ENABLE_TEMP_PREDICTION
        if (temp_predictor.isReady()) {
            snap.eta_to_brew_ready_ms = (temp < (brew_setpoint_c - 0.5f))
                ? temp_predictor.estimateTimeToTarget(brew_setpoint_c, 100.0f)
                : 0;
            snap.eta_to_steam_ready_ms = (temp < (steam_setpoint_c - 0.5f))
                ? temp_predictor.estimateTimeToTarget(steam_setpoint_c, 100.0f)
                : 0;
        }
        #endif

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
            xQueueOverwrite(queue_to_ui, &snap);
        }

        // Kick watchdog
        #if ENABLE_HW_WATCHDOG
        watchdog_kicked = true;
        #endif
    }
}
