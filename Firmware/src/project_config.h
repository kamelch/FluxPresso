#pragma once

// ===================================================================
// BOARD: Waveshare ESP32-S3-Touch-LCD-3.5B
// Display: AXS15231B QSPI 320×480 (same chip as JC3248W535EN)
// PMIC: AXP2101 (I2C 0x34)
// I/O Expander: TCA9554 (I2C 0x20) — handles LCD_RST, TP_INT, etc.
// Shared I2C bus: SDA=8, SCL=7 (touch, PMIC, IMU, RTC, codec)
// Camera: NOT USED — camera GPIOs repurposed for espresso peripherals
// ===================================================================

// --- DISPLAY SETTINGS (AXS15231B QSPI) - LANDSCAPE MODE ---
#define LCD_WIDTH       480
#define LCD_HEIGHT      320

// ===================================================================
// CORE BUS ADDRESSES / PINS
// (Referenced by hal.cpp)
// ===================================================================

// Shared I2C bus for touch / PMIC / I/O expander
#define I2C_SDA 8
#define I2C_SCL 7

// I2C device addresses on this board
#define TCA9554_ADDR 0x20
#define AXP2101_ADDR 0x34

// --- LCD BACKLIGHT ---
#define PIN_LCD_BL              6     // IO6 → LCD_BL (via S8050 NPN transistor)
#define LCD_BL_ACTIVE_HIGH      1     // HIGH = backlight on
#define LCD_BL_PWM_FREQ_HZ      20000
#define LCD_BL_PWM_CHANNEL      7
#define LCD_BL_PWM_RES_BITS     8

// ===================================================================
// PERIPHERAL PINS (Gaggia Espresso Machine)
// Using freed camera GPIOs on the 2×16 pin header (J8)
// Camera is NOT connected — these GPIOs are safe to repurpose.
//
// Pin header J8 (2×16) quick map for the espresso IOs (USB connector facing down):
//   Pin 7  = GPIO38  → AC_PWM (DIM)
//   Pin 9  = GPIO39  → AC_ZC
//   Pin 11 = GPIO40  → SOLENOID
//   Pin 13 = GPIO41  → SSR
//   Pin 15 = GPIO17  → HX_DT
//   Pin 17 = GPIO18  → HX_SCK
//   Pin 16 = GPIO42  → MAX_CS
//   Pin 22 = GPIO47  → MAX_DO
//   Pin 24 = GPIO48  → MAX_CLK
//   Pin 31/32 = 3V3, Pin 3/4/29/30 = GND
// ===================================================================

// --- TRIAC Dimmer (pump speed control) ---
#define PIN_AC_PWM      38    // IO38 (was CAM_XCLK) — TRIAC gate output
#define PIN_AC_ZC       39    // IO39 (was CAM_D6)   — Zero Cross input

// --- Solenoid (brew valve) ---
#define PIN_SOLENOID    40    // IO40 (was CAM_D5)
// Many relay modules are active-LOW: logic LOW = relay energized = solenoid OPEN.
// Set to true if your relay board closes the solenoid when the GPIO is LOW.
#define SOLENOID_ACTIVE_LOW  true

// --- SSR (boiler heater) ---
#define PIN_SSR         41    // IO41 (was CAM_PCLK)
#define SSR_ACTIVE_LOW   false  // Heater polarity: false=active-HIGH (common SSR), true=active-LOW (some relay boards)

// --- MAX31855 Thermocouple (SPI bit-bang) ---
#define PIN_MAX_CS      42    // IO42 (was CAM_D4)
#define PIN_MAX_DO      47    // IO47 (was CAM_D1)
#define PIN_MAX_CLK     48    // IO48 (was CAM_D2)

// --- HX711 Load Cell (scale) ---
#define PIN_HX_DT       17    // IO17 (was CAM_VSYNC)
#define PIN_HX_SCK      18    // IO18 (was CAM_HREF)

// --- SPARE GPIOS (on header, if needed later) ---
// IO21  (was CAM_D7)   — spare general purpose
// IO45  (was CAM_D0)   — strapping pin, use with care
// IO46  (was CAM_D3)   — strapping pin, use with care

// --- OPTIONAL HAPTIC FEEDBACK ---
#define ENABLE_HAPTIC   0
#define PIN_HAPTIC      -1

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

// --- ZERO-CROSS OPTIONS ---
#ifndef ENABLE_ZC
#define ENABLE_ZC 1
#endif
// If 0, ZC missing will be shown as a warning but will NOT trigger a STOP fault.
#ifndef FAULT_ON_ZC_MISSING
#define FAULT_ON_ZC_MISSING 0
#endif
#define GRID_FREQ_HZ            50    // Change to 60 for US/Canada

// ZC tolerance: allow brief ZC dropouts without blocking brew / faulting immediately.
#define ZC_MISS_GRACE_MS      300   // treat ZC as OK if seen within last 300ms
#define TRIAC_PULSE_US          200   // 200us gate pulse — more reliable triggering for inductive/vibratory pumps

// --- WIFI CONFIGURATION ---
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASSWORD   "YOUR_PASSWORD"

// --- OTA (WiFi Upload) ---
// First flash must be done via USB. After that you can upload over WiFi using PlatformIO espota.
#ifndef ENABLE_ARDUINO_OTA
#define ENABLE_ARDUINO_OTA 1
#endif
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "gaggia-e24"
#endif
// Optional: set a password (leave empty for no auth).
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif


// --- SHELLY SMART PLUG CONFIGURATION ---
#define SHELLY_ENABLED  true
#define SHELLY_IP       "192.168.1.100"
#define SHELLY_API_MODE 2
#define SHELLY_SWITCH_ID 0

// ===================================================================
// BREW TEMPERATURE PID CONTROLLER SETTINGS
// ===================================================================
#define BREW_PID_KP             25.0f
#define BREW_PID_KI             0.8f
#define BREW_PID_KD             8.0f
#define BREW_PID_DERIVATIVE_FILTER  0.15f
#define BREW_PID_OUTPUT_MIN     0.0f
#define BREW_PID_OUTPUT_MAX     100.0f
#define BREW_PID_SETPOINT_RAMP  1.5f

// ===================================================================
// STEAM TEMPERATURE PID CONTROLLER SETTINGS
// ===================================================================
#define STEAM_PID_KP            35.0f
#define STEAM_PID_KI            1.2f
#define STEAM_PID_KD            12.0f
#define STEAM_PID_DERIVATIVE_FILTER  0.2f
#define STEAM_PID_OUTPUT_MIN    0.0f
#define STEAM_PID_OUTPUT_MAX    100.0f
#define STEAM_PID_SETPOINT_RAMP 3.0f

// ===================================================================
// DREAMSTEAM CONTROLLER SETTINGS
// ===================================================================
#define DREAMSTEAM_DEFAULT_MODE  1
// More sensitive defaults so DreamSteam engages reliably during steaming.
#define DREAMSTEAM_TEMP_LOAD_THRESHOLD     1.0f
#define DREAMSTEAM_TEMP_DROP_RATE_THRESHOLD  -0.35f
#define DREAMSTEAM_PULSE_POWER_MIN         20
#define DREAMSTEAM_PULSE_POWER_MAX         40
#define DREAMSTEAM_PULSE_DURATION_MS       180
#define DREAMSTEAM_PULSE_INTERVAL_MS       1000
#define DREAMSTEAM_ADAPTIVE_GAIN           0.3f
#define DREAMSTEAM_ADAPTIVE_WINDOW_MS      5000
#define DREAMSTEAM_MAX_PUMP_MS_PER_MIN     10000
#define DREAMSTEAM_MIN_REFILL_TEMP         130.0f
#define DREAMSTEAM_MAX_REFILL_TEMP         150.0f
#define DREAMSTEAM_MAX_CONTINUOUS_PULSES   8
#define DREAMSTEAM_USE_VALVE_SWITCH        false

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

// Time (ms) to wait in PREHEAT after Shelly turns ON, for relay + ZC to stabilize.
// Without this, the system faults on ZC_MISSING because the dimmer has no AC yet.
#define SHELLY_STABILIZE_MS     1500

// Hard temperature ceiling in brew mode. If temp exceeds this, heater is killed
// regardless of PID output. Prevents thermal runaway from integral windup.
#define BREW_HARD_TEMP_LIMIT_C  105.0f

// Soft-landing: reduce heater power proportionally when near setpoint.
// Prevents overshoot from the Gaggia boiler's thermal inertia.
#define BREW_GLIDE_RANGE_C      5.0f   // start reducing power within this range
#define BREW_GLIDE_MAX_DUTY     40     // max duty% when within glide range
#define BREW_CLOSE_RANGE_C      2.0f   // "close enough" zone
#define BREW_CLOSE_MAX_DUTY     16     // max duty% when very close

// --- WATCHDOG ---
#define ENABLE_HW_WATCHDOG      false  // DISABLED FOR DEBUG
#define WATCHDOG_TIMEOUT_MS     500

// --- UI FEATURES ---
#define ENABLE_SHOT_HISTORY     true
#define MAX_SHOT_HISTORY        20
#define ENABLE_AUTO_TARE        true
#define CHART_HISTORY_SECONDS   120
#define CHART_UPDATE_RATE_HZ    2
#define CHART_POINT_COUNT       240

// --- ADVANCED FEATURES ---
#define ENABLE_PID_DIAGNOSTICS  true
#define ENABLE_TEMP_PREDICTION  true
#define ENABLE_ADAPTIVE_PREHEAT true
#define ENABLE_SENSOR_FILTERING true
#define ENABLE_PUMP_RAMPING     true    // Phase-controlled TRIAC dimmer: ramping smooths pre/full/decline transitions
#define ENABLE_SHOT_PERSISTENCE true

// --- FLOW SAFETY ---
#define ENABLE_FLOW_STALL_DETECTION false  // DISABLED — too sensitive, causes false faults

// --- SCALE CALIBRATION ---
#define DEFAULT_SCALE_CALIBRATION   -420.0f
#define DEFAULT_SCALE_OFFSET        0L

// --- FLOW / EXTRACTION DIAGNOSTICS ---
#define ENABLE_FLOW_STALL_DETECT   false  // legacy alias, unused
#define FLOW_STALL_TIMEOUT_MS      12000
#define FLOW_STALL_MIN_PROGRESS_G  0.3f
#define FLOW_STALL_MIN_PUMP_PCT    20

#define ENABLE_PID_SATURATION_FAULT false  // DISABLED — Gaggia boiler is slow, PID saturation is normal during heat-up
#define PID_SATURATION_TIMEOUT_MS   90000
#define PID_SATURATION_MIN_RISE_C   2.0f

// --- SENSOR FILTERING ---
#define TEMP_FILTER_WINDOW      5
#define WEIGHT_FILTER_WINDOW    5
#define USE_MEDIAN_FILTER       true

// --- PUMP RAMPING ---
#define PUMP_RAMP_UP_MS         2500
#define PUMP_RAMP_DOWN_MS       2500
#define PUMP_MIN_STEP_PERCENT   2

// --- UI OPTIMIZATION ---
#define UI_UPDATE_THRESHOLD_TEMP   0.1f
#define UI_UPDATE_THRESHOLD_WEIGHT 0.5f
#define UI_FORCE_UPDATE_MS         1000

// --- OVERCURRENT PROTECTION ---
#define ENABLE_OVERCURRENT_PROTECTION false  // DISABLED — Gaggia runs heater+pump together by design
#define MAX_HEATER_PUMP_OVERLAP_MS    2000
#define HEATER_PRIORITY_MODE          false

// --- OPTIONAL CONNECTIVITY FEATURES ---
#ifndef ENABLE_BLE_PROVISIONING
#define ENABLE_BLE_PROVISIONING 0
#endif
#ifndef ENABLE_OTA_UPDATES
#define ENABLE_OTA_UPDATES 0
#endif
#ifndef OTA_MANIFEST_URL
#define OTA_MANIFEST_URL ""
#endif
#ifndef OTA_HTTPS_FINGERPRINT
#define OTA_HTTPS_FINGERPRINT ""
#endif
