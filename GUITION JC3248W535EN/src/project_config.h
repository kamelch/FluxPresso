#pragma once

// --- DISPLAY SETTINGS (JC3248W535EN QSPI) - LANDSCAPE MODE ---
#define LCD_WIDTH       480
#define LCD_HEIGHT      320

// --- PERIPHERAL PINS (Gaggia Setup) ---
// P1 Header
#define PIN_AC_PWM      43    // P1 TX - Triac Gate
#define PIN_AC_ZC       44    // P1 RX - Zero Cross Input

// P2 Header
#define PIN_SOLENOID    5     // P2 IO5
#define PIN_MAX_CS      6     // P2 IO6
#define PIN_MAX_DO      7     // P2 IO7
#define PIN_MAX_CLK     15    // P2 IO15
#define PIN_SSR         16    // P2 IO16

// P3 Header
#define PIN_HX_DT       17    // P3 IO17

// --- OPTIONAL HAPTIC FEEDBACK (requires vibration motor + driver) ---
#define ENABLE_HAPTIC   0
// Set to a valid GPIO if you wired a vibration motor driver; otherwise keep -1
#define PIN_HAPTIC      -1
#define PIN_HX_SCK      18    // P3 IO18

// Optional: Steam valve detection microswitch
#define PIN_STEAM_VALVE  0    // 0 = not connected (use temp-based detection)
#define STEAM_VALVE_ACTIVE_LOW false

// --- SAFETY LIMITS ---
#define SAFETY_MAX_TEMP_C       165.0
#define SAFETY_MAX_SHOT_TIME_S  60
#define SAFETY_MIN_FLOW_RATE    0.1

// --- TASK CONFIGURATION ---
#define CORE_TASK_RATE_MS       50    // 20Hz Control Loop
#define UI_TASK_RATE_MS         50    // 20Hz UI Refresh
#define NET_TASK_RATE_MS        200   // 5Hz Network Check

// --- DIMMER CONFIG ---
#define GRID_FREQ_HZ            50    // Change to 60 for US/Canada
#define TRIAC_PULSE_US          20

// --- WIFI CONFIGURATION ---
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASSWORD   "YOUR_PASSWORD"

// --- SHELLY SMART PLUG CONFIGURATION ---
#define SHELLY_ENABLED  true
#define SHELLY_IP       "192.168.1.100"

// ===================================================================
// BREW TEMPERATURE PID CONTROLLER SETTINGS
// ===================================================================
// Tuned for stable espresso brewing with minimal overshoot
#define BREW_PID_KP             25.0f   // Proportional gain
#define BREW_PID_KI             0.8f    // Integral gain (anti-windup enabled)
#define BREW_PID_KD             8.0f    // Derivative gain (with filtering)
#define BREW_PID_DERIVATIVE_FILTER  0.15f  // Low-pass filter (0.1-0.2 typical)
#define BREW_PID_OUTPUT_MIN     0.0f    // Minimum heater output
#define BREW_PID_OUTPUT_MAX     100.0f  // Maximum heater output
#define BREW_PID_SETPOINT_RAMP  1.5f    // Max °C/s setpoint change (smooth transitions)

// ===================================================================
// STEAM TEMPERATURE PID CONTROLLER SETTINGS
// ===================================================================
// More aggressive tuning for faster steam warmup
#define STEAM_PID_KP            35.0f
#define STEAM_PID_KI            1.2f
#define STEAM_PID_KD            12.0f
#define STEAM_PID_DERIVATIVE_FILTER  0.2f
#define STEAM_PID_OUTPUT_MIN    0.0f
#define STEAM_PID_OUTPUT_MAX    100.0f
#define STEAM_PID_SETPOINT_RAMP 3.0f    // Faster ramp for steam mode

// ===================================================================
// DREAMSTEAM CONTROLLER SETTINGS
// ===================================================================
#define DREAMSTEAM_DEFAULT_MODE  1  // 0=MODE_DISABLED, 1=MODE_CONSERVATIVE, 2=MODE_AGGRESSIVE, 3=MODE_ADAPTIVE

// Temperature-based steam load detection
#define DREAMSTEAM_TEMP_LOAD_THRESHOLD     2.0f   // °C below setpoint
#define DREAMSTEAM_TEMP_DROP_RATE_THRESHOLD  -0.8f  // °C/s

// Pump pulse parameters
#define DREAMSTEAM_PULSE_POWER_MIN         20    // Minimum pump power %
#define DREAMSTEAM_PULSE_POWER_MAX         40    // Maximum pump power %
#define DREAMSTEAM_PULSE_DURATION_MS       180   // Individual pulse length
#define DREAMSTEAM_PULSE_INTERVAL_MS       1000  // Minimum time between pulses

// Adaptive tuning
#define DREAMSTEAM_ADAPTIVE_GAIN           0.3f  // Learning rate
#define DREAMSTEAM_ADAPTIVE_WINDOW_MS      5000  // Evaluation window

// Safety limits
#define DREAMSTEAM_MAX_PUMP_MS_PER_MIN     10000  // 10s total per minute
#define DREAMSTEAM_MIN_REFILL_TEMP         130.0f // Don't refill below this
#define DREAMSTEAM_MAX_REFILL_TEMP         150.0f // Don't refill above this
#define DREAMSTEAM_MAX_CONTINUOUS_PULSES   8      // Prevent runaway

// Optional steam valve switch
#define DREAMSTEAM_USE_VALVE_SWITCH        false  // Set true if microswitch installed

// --- TEMPERATURE DEFAULTS ---
#define DEFAULT_BREW_TEMP_C     93.0f
#define DEFAULT_STEAM_TEMP_C    145.0f

// --- MANUAL PUMP SAFETY ---
#define MANUAL_PUMP_TIMEOUT_MS  60000
#define MANUAL_PUMP_WARNING_MS  30000
#define MANUAL_PUMP_MAX_TEMP    110.0f

// --- BREW TIMINGS ---
#define DRAIN_TIME_MS           1500
#define MANUAL_DRAIN_TIME_MS    800
#define FLUSH_TIME_MS           5000
#define PREHEAT_TEMP_OFFSET_C   5.0f

// --- WATCHDOG ---
#define ENABLE_HW_WATCHDOG      true   // Hardware watchdog for safety
#define WATCHDOG_TIMEOUT_MS     500

// --- UI FEATURES ---
#define ENABLE_SHOT_HISTORY     true
#define MAX_SHOT_HISTORY        20
#define ENABLE_AUTO_TARE        true
#define CHART_HISTORY_SECONDS   120
#define CHART_UPDATE_RATE_HZ    2      // Chart update frequency
#define CHART_POINT_COUNT       240    // 120s * 2Hz = 240 points

// --- ADVANCED FEATURES ---
#define ENABLE_PID_DIAGNOSTICS  true   // Send detailed PID data to UI
#define ENABLE_TEMP_PREDICTION  true   // Predictive temperature modeling
#define ENABLE_ADAPTIVE_PREHEAT true   // Learn optimal preheat timing
#define ENABLE_SENSOR_FILTERING true   // 5-sample moving average filters
#define ENABLE_PUMP_RAMPING     true   // Smooth pump power transitions
#define ENABLE_SHOT_PERSISTENCE true   // Save shot history to NVS

// --- SENSOR FILTERING ---
#define TEMP_FILTER_WINDOW      5      // Moving average window
#define WEIGHT_FILTER_WINDOW    5      // Moving average window
#define USE_MEDIAN_FILTER       false  // Use median filter instead (spike rejection)

// --- PUMP RAMPING ---
#define PUMP_RAMP_UP_MS         150    // 0 → 100% ramp time
#define PUMP_RAMP_DOWN_MS       100    // 100% → 0 ramp time
#define PUMP_MIN_STEP_PERCENT   2      // Minimum power change per step

// --- UI OPTIMIZATION ---
#define UI_UPDATE_THRESHOLD_TEMP   0.1f   // Only update if temp changes by ±0.1°C
#define UI_UPDATE_THRESHOLD_WEIGHT 0.5f   // Only update if weight changes by ±0.5g
#define UI_FORCE_UPDATE_MS         1000   // Force update every 1 second minimum

// --- OVERCURRENT PROTECTION ---
#define ENABLE_OVERCURRENT_PROTECTION true
#define MAX_HEATER_PUMP_OVERLAP_MS    2000  // Max time heater+pump can run together
#define HEATER_PRIORITY_MODE          false // If true, reduce pump when heater on