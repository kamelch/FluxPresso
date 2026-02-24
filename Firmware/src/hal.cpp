#include "hal.h"
#include "project_config.h"

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Wire.h>
#include <Arduino_GFX_Library.h>
// These specific includes are required for GFX Library for Arduino v1.4.7
#include <databus/Arduino_ESP32QSPI.h>
#include <display/Arduino_AXS15231B.h>
// Canvas provides a full PSRAM framebuffer — required because the AXS15231B
// QSPI variant has BROKEN partial window addressing (CASET/RASET ignored).
#include <canvas/Arduino_Canvas.h>

#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_timer.h"
#include "esp_system.h"

#include <Adafruit_MAX31855.h>
#include "HX711.h"

// ===================================================================
// WAVESHARE ESP32-S3-Touch-LCD-3.5B — Display & I/O Configuration
// ===================================================================

// --- QSPI Display Pins (from Waveshare schematic) ---
// NOTE: Do NOT rely on Arduino_GFX "GFX_QSPI_*" macros; those are not
// defined for generic PlatformIO devkit targets. Use explicit GPIOs.
#define WS_QSPI_CS    12
#define WS_QSPI_SCK    5
#define WS_QSPI_D0     1
#define WS_QSPI_D1     2
#define WS_QSPI_D2     3
#define WS_QSPI_D3     4
#define WS_RST        -1             // Reset via TCA9554, not direct GPIO

// AXS15231B touch controller (same chip as display, touch via I2C at 0x3B)
static constexpr uint8_t AXS_TOUCH_ADDR = 0x3B;
static const uint8_t AXS_READ_TOUCHPAD[] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};
static constexpr int LCD_ROTATION = 1;

// Physical panel dimensions (portrait native)
#define PANEL_W  320
#define PANEL_H  480

// ===================================================================
// TCA9554 I/O Expander (I2C address 0x20)
// Minimal driver — just enough for LCD reset. No external library needed.
// ===================================================================
#define TCA9554_REG_INPUT    0x00
#define TCA9554_REG_OUTPUT   0x01
#define TCA9554_REG_POLARITY 0x02
#define TCA9554_REG_CONFIG   0x03  // 0=output, 1=input (default all input)

static uint8_t tca9554_output_state = 0x00;  // track current output register

static bool tca9554_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static uint8_t tca9554_read_reg(uint8_t reg) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)TCA9554_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// Set a single EXIO pin direction (0=output, 1=input)
static void tca9554_pin_mode(uint8_t bit, bool is_input) {
    uint8_t cfg = tca9554_read_reg(TCA9554_REG_CONFIG);
    if (is_input) cfg |= (1 << bit);
    else          cfg &= ~(1 << bit);
    tca9554_write_reg(TCA9554_REG_CONFIG, cfg);
}

// Set a single EXIO output pin HIGH or LOW
static void tca9554_write_pin(uint8_t bit, bool high) {
    if (high) tca9554_output_state |= (1 << bit);
    else      tca9554_output_state &= ~(1 << bit);
    tca9554_write_reg(TCA9554_REG_OUTPUT, tca9554_output_state);
}

// ===================================================================
// AXP2101 PMIC — Minimal init to ensure display power rails are ON
// The Waveshare board uses AXP2101 for all power management.
// Default power-on state is usually correct, but we verify key rails.
// ===================================================================
#define AXP2101_REG_STATUS1     0x00
#define AXP2101_REG_ONOFF_CTL1  0x90  // DCDC1-5 enable
#define AXP2101_REG_ONOFF_CTL2  0x91  // ALDO1-4, BLDO1-2 enable
#define AXP2101_REG_DCDC1_VOLT  0x82
#define AXP2101_REG_ALDO1_VOLT  0x92
#define AXP2101_REG_ALDO2_VOLT  0x93
#define AXP2101_REG_ALDO3_VOLT  0x94
#define AXP2101_REG_ALDO4_VOLT  0x95

static bool axp2101_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static uint8_t axp2101_read_reg(uint8_t reg) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AXP2101_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static void axp2101_init() {
    // Read current PMIC status for diagnostics
    uint8_t status = axp2101_read_reg(AXP2101_REG_STATUS1);
    Serial.printf("[AXP2101] Status1: 0x%02X\n", status);

    // Ensure DCDC1 (VCC3V3 main rail) is enabled
    uint8_t onoff1 = axp2101_read_reg(AXP2101_REG_ONOFF_CTL1);
    Serial.printf("[AXP2101] ONOFF_CTL1: 0x%02X\n", onoff1);
    if (!(onoff1 & 0x01)) {
        Serial.println("[AXP2101] WARNING: DCDC1 was OFF, enabling...");
        axp2101_write_reg(AXP2101_REG_ONOFF_CTL1, onoff1 | 0x01);
    }

    // Ensure ALDO outputs are enabled (used for various peripherals)
    uint8_t onoff2 = axp2101_read_reg(AXP2101_REG_ONOFF_CTL2);
    Serial.printf("[AXP2101] ONOFF_CTL2: 0x%02X (ALDO/BLDO enables)\n", onoff2);

    // The Waveshare board's default AXP2101 OTP (one-time programmed) settings
    // should have the correct voltages. We just verify and log.
    uint8_t dcdc1_v = axp2101_read_reg(AXP2101_REG_DCDC1_VOLT);
    Serial.printf("[AXP2101] DCDC1 voltage reg: 0x%02X\n", dcdc1_v);

    Serial.println("[AXP2101] PMIC init complete");
}

// ===================================================================
// Display Driver Objects
// ===================================================================
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    WS_QSPI_CS, WS_QSPI_SCK, WS_QSPI_D0, WS_QSPI_D1, WS_QSPI_D2, WS_QSPI_D3
);

// Raw display driver — rotation=0 always (software rotation in flush)
Arduino_AXS15231B *display = new Arduino_AXS15231B(
    bus, WS_RST, 0 /* always portrait */, false /* IPS */, PANEL_W, PANEL_H
);

// Canvas wraps the raw driver with a full PSRAM framebuffer (320×480×2 = 307KB).
// Waveshare has 8MB PSRAM — plenty of room.
Arduino_Canvas *gfx = new Arduino_Canvas(PANEL_W, PANEL_H, display, 0, 0, 0);

// --- SENSORS & ACTUATORS ---
Adafruit_MAX31855 thermocouple(PIN_MAX_CLK, PIN_MAX_CS, PIN_MAX_DO);
HX711 scale;

// --- Scale calibration ---
static float g_scale_factor = 1.0f;
static long  g_scale_offset = 0;

// Persistent last-known-good sensor values
static float g_last_weight_g = 0.0f;
static bool  g_last_zc_health = true;

// --- DIMMER VARIABLES ---
static TaskHandle_t g_dimmer_task = nullptr;
static esp_timer_handle_t g_triac_timer = nullptr;
volatile uint32_t dimming_delay_us = 0;
volatile bool pump_active = false;

// ZC health tracking (glitch-filtered)
static constexpr uint32_t ZC_GLITCH_REJECT_US = 3000;
static volatile uint32_t zc_count = 0;
static uint32_t last_zc_count = 0;
static uint32_t last_zc_check_ms = 0;
static volatile uint32_t zc_last_accepted_us = 0;
static volatile uint32_t zc_period_us = 0;
static uint32_t zc_ema_us = 0;
static volatile uint32_t zc_jitter_score = 0;

// Slow PWM for SSR heater
static uint32_t heater_pwm_epoch_ms = 0;

// Scale timeout tracking
static uint32_t last_scale_ok_ms = 0;
static bool     g_scale_stale = true;

// --- TRIAC fire callback (runs in esp_timer task context) ---
static void triac_fire_cb(void* arg) {
    (void)arg;
    gpio_set_level((gpio_num_t)PIN_AC_PWM, 1);
    // Short busy-wait is OK here (200us default)
    delayMicroseconds(TRIAC_PULSE_US);
    gpio_set_level((gpio_num_t)PIN_AC_PWM, 0);
}

void IRAM_ATTR HAL_ISR_DimmerTimer() {
    // Deprecated path (kept for header compatibility).
}

void IRAM_ATTR HAL_ISR_ZeroCross() {
    const uint32_t now_us = (uint32_t)esp_timer_get_time();
    const uint32_t last_us = zc_last_accepted_us;

    // Reject very-fast re-triggers (noise / opto bounce)
    if (last_us != 0) {
        const uint32_t dt = (uint32_t)(now_us - last_us);
        if (dt < ZC_GLITCH_REJECT_US) {
            return;
        }
        zc_period_us = dt;

        // EMA + jitter score (decaying)
        if (zc_ema_us == 0) zc_ema_us = dt;
        const int32_t diff = (int32_t)dt - (int32_t)zc_ema_us;
        zc_ema_us = (uint32_t)((int32_t)zc_ema_us + diff / 8);
        const uint32_t adiff = (diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff;
        if (adiff > 1200) {
            // Bad pulse period: increase score quickly
            if (zc_jitter_score < 250) zc_jitter_score += 8;
        } else {
            // Good pulse: decay slowly
            if (zc_jitter_score > 0) zc_jitter_score -= 1;
        }
    }

    zc_last_accepted_us = now_us;
    zc_count++;

    if (pump_active && dimming_delay_us > 0 && g_dimmer_task) {
        BaseType_t hpw = pdFALSE;
        xTaskNotifyFromISR(g_dimmer_task, dimming_delay_us, eSetValueWithOverwrite, &hpw);
        if (hpw) portYIELD_FROM_ISR();
    }
}

// Dimmer service task (arms the one-shot triac pulse each half-cycle)
static void dimmer_service_task(void* pv) {
    (void)pv;
    for (;;) {
        uint32_t delay_us = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &delay_us, portMAX_DELAY);
        if (!pump_active || delay_us == 0) continue;

        if (!g_triac_timer) {
            // Fallback: constant HIGH (most opto-triac dimmer modules conduct at the next crossing)
            gpio_set_level((gpio_num_t)PIN_AC_PWM, 1);
            continue;
        }

        (void)esp_timer_stop(g_triac_timer);
        (void)esp_timer_start_once(g_triac_timer, (uint64_t)delay_us);
    }
}

// --- INIT ---
void HAL_ForceOutputsSafeEarly() {
    pinMode(PIN_SOLENOID, OUTPUT);
    pinMode(PIN_SSR, OUTPUT);
    pinMode(PIN_AC_PWM, OUTPUT);
    // Solenoid CLOSED at boot (active-LOW: HIGH=closed, active-HIGH: LOW=closed)
    #if SOLENOID_ACTIVE_LOW
    digitalWrite(PIN_SOLENOID, HIGH);
    #else
    digitalWrite(PIN_SOLENOID, LOW);
    #endif
    // Heater OFF at boot (respect SSR polarity)
    #if SSR_ACTIVE_LOW
    digitalWrite(PIN_SSR, HIGH);  // HIGH = OFF for active-LOW boards
    #else
    digitalWrite(PIN_SSR, LOW);   // LOW  = OFF for active-HIGH boards
    #endif
    digitalWrite(PIN_AC_PWM, LOW);
}

uint32_t HAL_GetResetReason() {
    return (uint32_t)esp_reset_reason();
}

bool HAL_Init() {
    bool ok = true;

    // 1. Outputs
    pinMode(PIN_SOLENOID, OUTPUT);
    #if SOLENOID_ACTIVE_LOW
    digitalWrite(PIN_SOLENOID, HIGH);  // HIGH = closed for active-LOW relay
    #else
    digitalWrite(PIN_SOLENOID, LOW);   // LOW = closed for direct drive
    #endif
    pinMode(PIN_SSR, OUTPUT);
    // Ensure heater is OFF at boot (polarity depends on your SSR/relay module)
    #if SSR_ACTIVE_LOW
    digitalWrite(PIN_SSR, HIGH);  // HIGH = OFF for active-LOW SSR/relay
    #else
    digitalWrite(PIN_SSR, LOW);   // LOW  = OFF for active-HIGH SSR
    #endif
    pinMode(PIN_AC_PWM, OUTPUT);   digitalWrite(PIN_AC_PWM, LOW);
    
    // 2. Inputs
    pinMode(PIN_AC_ZC, INPUT_PULLUP);
    
    // 3. Sensors
    if (!thermocouple.begin()) {
        Serial.println("[HAL] WARNING: thermocouple.begin() failed — check MAX31855 wiring (CS=42 DO=47 CLK=48)");
        // Don't fail init — the control loop will handle NaN readings gracefully
    }
    scale.begin(PIN_HX_DT, PIN_HX_SCK);

    scale.set_scale(g_scale_factor);
    scale.set_offset(g_scale_offset);

    heater_pwm_epoch_ms = millis();
    last_scale_ok_ms = millis();

    // 4. TRIAC gate pulse timer + dimmer service task
    {
        esp_timer_create_args_t targs = {};
        targs.callback = &triac_fire_cb;
        targs.arg = NULL;
        targs.dispatch_method = ESP_TIMER_TASK;
        targs.name = "triac";
        const esp_err_t err = esp_timer_create(&targs, &g_triac_timer);
        if (err != ESP_OK) {
            Serial.printf("[HAL] WARNING: esp_timer_create(triac) failed: %d\n", (int)err);
            Serial.println("[HAL] Falling back to constant gate HIGH mode");
            g_triac_timer = nullptr;
        }

        const BaseType_t created = xTaskCreatePinnedToCore(dimmer_service_task, "DimmerSvc", 2048, NULL, 4, &g_dimmer_task, 1);
        if (created != pdPASS) {
            Serial.println("[HAL] WARNING: Failed to create DimmerSvc task — pump dimmer will not work");
            g_dimmer_task = nullptr;
        }
    }

    attachInterrupt(digitalPinToInterrupt(PIN_AC_ZC), HAL_ISR_ZeroCross, RISING);

    // Quick sanity read — log but don't fail
    double c = thermocouple.readCelsius();
    if (isnan(c)) {
        Serial.printf("[HAL] WARNING: Initial thermocouple read = NaN (will retry in control loop)\n");
        // NOT setting ok=false — let the control loop handle this gracefully
    } else {
        Serial.printf("[HAL] Thermocouple initial read: %.1f°C\n", c);
    }
    return ok;
}

void HAL_ReadSensors(float &temp, float &weight, bool &zc_health) {
    // Temp
    double c = thermocouple.readCelsius();
    temp = isnan(c) ? 999.0f : (float)c;

    // Weight — guarded with timeout.
// HX711 library's read() can busy-wait on DOUT. We avoid calling it unless is_ready().
// If readings become slow during pump/dimmer activity, we KEEP the last good value and mark it stale,
// instead of forcing weight to -999 (which would disable weight-stop and fall back to time-stop).
if (scale.is_ready()) {
    const uint32_t t0 = millis();
    float new_w = scale.get_units(1);
    const uint32_t elapsed_read = millis() - t0;

    const bool w_ok = isfinite(new_w) && new_w > -5000.0f && new_w < 5000.0f;
    if (w_ok) {
        g_last_weight_g = new_w;
        last_scale_ok_ms = millis();
        if (elapsed_read >= 200) {
            Serial.printf("[HAL] WARNING: HX711 read took %lums (slow) — check wiring/EMI\n", (unsigned long)elapsed_read);
        }
    } else if (elapsed_read >= 200) {
        Serial.printf("[HAL] WARNING: HX711 read took %lums and returned invalid value — check wiring/EMI\n", (unsigned long)elapsed_read);
    }
}

const uint32_t age_ms = (last_scale_ok_ms == 0) ? 0xFFFFFFFFu : (millis() - last_scale_ok_ms);
g_scale_stale = (last_scale_ok_ms == 0) || (age_ms > 8000);
weight = (last_scale_ok_ms == 0) ? -999.0f : g_last_weight_g;
    // ZC health check (glitch filtered + grace window)
    const uint32_t now_ms = millis();
    if ((now_ms - last_zc_check_ms) >= 100) {
        const uint32_t ccount = zc_count;
        const uint32_t now_us = (uint32_t)esp_timer_get_time();
        const uint32_t last_us = zc_last_accepted_us;

        const uint32_t grace_us = (uint32_t)(ZC_MISS_GRACE_MS * 1000UL);
        const bool recent = (last_us != 0) && ((uint32_t)(now_us - last_us) <= grace_us);

        bool zc_ok = recent;
        const uint32_t p = zc_period_us;
        if (p != 0) {
            // 50Hz half-wave ~= 10000us, 60Hz half-wave ~= 8333us
            if (p < 7000 || p > 13000) zc_ok = false;
        } else {
            // No period yet: require at least one pulse since last check
            if (ccount == last_zc_count) zc_ok = false;
        }

        // Jitter score is a decaying metric (0..255).
        if (zc_jitter_score > 80) zc_ok = false;

        g_last_zc_health = zc_ok;
        last_zc_count = ccount;
        last_zc_check_ms = now_ms;
    }
    zc_health = g_last_zc_health;
}

void HAL_TareScale() {
    // Reduced from tare(10) → tare(3) to minimize control loop blocking.
    // tare(10) took ~1 second — the heater/pump ran uncontrolled during that time.
    // tare(3) takes ~300ms: still a compromise, but much safer.
    scale.tare(3);
    g_scale_offset = scale.get_offset();
}

void HAL_SetScaleCalibration(float scale_factor) {
    if (!isfinite(scale_factor) || fabsf(scale_factor) < 0.0001f) return;
    g_scale_factor = scale_factor;
    scale.set_scale(g_scale_factor);
}

float HAL_GetScaleCalibration() {
    return scale.get_scale();
}

void HAL_SetScaleOffset(long offset) {
    g_scale_offset = offset;
    scale.set_offset(g_scale_offset);
}

long HAL_GetScaleOffset() {
    return scale.get_offset();
}

void HAL_SetOutputs(uint8_t pump_percent, bool solenoid_open, uint8_t heater_duty) {
    if (pump_percent > 100) pump_percent = 100;
    if (heater_duty > 100) heater_duty = 100;

    // Solenoid — handle active-LOW relay modules.
    // Active-LOW: GPIO LOW = relay energized = solenoid OPEN.
    // Active-HIGH: GPIO HIGH = solenoid OPEN (direct drive).
    #if SOLENOID_ACTIVE_LOW
    digitalWrite(PIN_SOLENOID, solenoid_open ? LOW : HIGH);
    #else
    digitalWrite(PIN_SOLENOID, solenoid_open ? HIGH : LOW);
    #endif

    // Slow PWM (time proportioning) for SSR
    const uint32_t HEATER_PERIOD_MS = 1000;
    const uint32_t now = millis();
    const uint32_t phase = (now - heater_pwm_epoch_ms) % HEATER_PERIOD_MS;
    const uint32_t on_ms = (uint32_t)heater_duty * HEATER_PERIOD_MS / 100;
    const bool ssr_on = (heater_duty > 0 && phase < on_ms);
    #if SSR_ACTIVE_LOW
    digitalWrite(PIN_SSR, ssr_on ? LOW : HIGH);
    #else
    digitalWrite(PIN_SSR, ssr_on ? HIGH : LOW);
    #endif

    // Pump Dimmer
    if (pump_percent == 0) {
        pump_active = false;
        dimming_delay_us = 0;
        if (g_triac_timer) (void)esp_timer_stop(g_triac_timer);
        gpio_set_level((gpio_num_t)PIN_AC_PWM, 0);
    } else {
        pump_active = true;
        const uint32_t half_wave_us = 1000000UL / (GRID_FREQ_HZ * 2UL);
        const int MIN_DELAY_US = 200;
        const int MAX_DELAY_US = (int)half_wave_us - TRIAC_PULSE_US - 100;
        int delay = MIN_DELAY_US + ((100 - (int)pump_percent) * (MAX_DELAY_US - MIN_DELAY_US) / 100);
        if (delay < MIN_DELAY_US) delay = MIN_DELAY_US;
        if (delay > MAX_DELAY_US) delay = MAX_DELAY_US;
        dimming_delay_us = (uint32_t)delay;
    }
}

void HAL_AllOutputsOff() {
    HAL_SetOutputs(0, false, 0);
}

// ===================================================================
// DISPLAY ADAPTER — Same rotation logic as JC3248W535EN
// The AXS15231B has identical behavior on both boards.
// ===================================================================
static uint32_t flush_count = 0;

void HAL_Display_Flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
    uint16_t *canvas_fb = (uint16_t *)gfx->getFramebuffer();
    if (!canvas_fb) {
        lv_disp_flush_ready(disp);
        return;
    }

    const uint16_t *src = (const uint16_t *)color_p;
    const int32_t lw = area->x2 - area->x1 + 1;
    const int32_t lh = area->y2 - area->y1 + 1;

    if (flush_count < 3) {
        Serial.printf("[FLUSH #%lu] LVGL(%d,%d)-(%d,%d) %dx%d\n",
            flush_count, area->x1, area->y1, area->x2, area->y2, (int)lw, (int)lh);
    }

    // Software 90° CW rotation: LVGL landscape → Canvas portrait framebuffer
    for (int32_t row = 0; row < lh; row++) {
        const int32_t abs_y = area->y1 + row;
        const int32_t cx_base = (PANEL_W - 1) - abs_y;
        for (int32_t col = 0; col < lw; col++) {
            const int32_t abs_x = area->x1 + col;
            const int32_t cy = abs_x;
            canvas_fb[cy * PANEL_W + cx_base] = src[row * lw + col];
        }
    }

    if (lv_disp_flush_is_last(disp)) {
        esp_task_wdt_reset();
        gfx->flush();
        if (flush_count < 3) {
            Serial.println("[FLUSH] Full frame pushed to display via Canvas::flush()");
        }
    }

    flush_count++;
    lv_disp_flush_ready(disp);
}

void HAL_Touch_Read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    (void)indev_driver;
    static uint32_t touch_log_count = 0;

    // AXS15231B touch read via I2C (address 0x3B)
    uint8_t buf[8] = {0};
    
    Wire.beginTransmission(AXS_TOUCH_ADDR);
    Wire.write(AXS_READ_TOUCHPAD, sizeof(AXS_READ_TOUCHPAD));
    uint8_t err = Wire.endTransmission(false);
    
    if (err != 0) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    
    uint8_t got = Wire.requestFrom(AXS_TOUCH_ADDR, (uint8_t)8);
    if (got < 8) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    
    for (int i = 0; i < 8; i++) {
        buf[i] = Wire.read();
    }
    
    if (buf[0] != 0) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    
    uint16_t tx = ((uint16_t)(buf[2] & 0x0F) << 8) | buf[3];
    uint16_t ty = ((uint16_t)(buf[4] & 0x0F) << 8) | buf[5];

    if (touch_log_count < 20 || (touch_log_count % 100) == 0) {
        Serial.printf("[TOUCH] raw: tx=%u ty=%u buf=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
            tx, ty, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    }
    touch_log_count++;

    // Map portrait touch coords to LVGL landscape (480x320)
    uint16_t x = ty;
    uint16_t y = (PANEL_W - 1) - tx;

    if (x >= LCD_WIDTH)  x = LCD_WIDTH - 1;
    if (y >= LCD_HEIGHT) y = LCD_HEIGHT - 1;

    if (touch_log_count <= 20) {
        Serial.printf("[TOUCH] mapped: x=%u y=%u\n", x, y);
    }

    data->state = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;
}


// ===================================================================
// HAL_Display_Init — Waveshare-specific initialization sequence
// Key differences from JC3248W535EN:
//   1. Shared I2C bus (SDA=8, SCL=7) for touch + PMIC + IMU + RTC
//   2. TCA9554 I/O expander handles LCD_RST (EXIO1)
//   3. AXP2101 PMIC must be verified/initialized
//   4. Backlight on GPIO6 (was GPIO1 on JC3248W535EN)
// ===================================================================
void HAL_Display_Init() {
    Serial.println("[HAL] === Waveshare ESP32-S3-Touch-LCD-3.5B Init ===");

    // --- Step 1: Initialize shared I2C bus ---
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // 400kHz for I2C devices
    Serial.printf("[HAL] I2C initialized: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);

    // --- Step 2: I2C bus scan (diagnostic) ---
    Serial.println("[HAL] I2C scan:");
    bool found_tca = false, found_axp = false, found_touch = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[HAL]   0x%02X", addr);
            if (addr == TCA9554_ADDR)  { Serial.print(" (TCA9554 I/O expander)"); found_tca = true; }
            if (addr == AXP2101_ADDR)  { Serial.print(" (AXP2101 PMIC)"); found_axp = true; }
            if (addr == AXS_TOUCH_ADDR){ Serial.print(" (AXS15231 touch)"); found_touch = true; }
            if (addr == 0x51)          Serial.print(" (PCF85063 RTC)");
            if (addr == 0x6B)          Serial.print(" (QMI8658 IMU)");
            if (addr == 0x18)          Serial.print(" (ES8311 codec)");
            Serial.println();
        }
    }

    // --- Step 3: AXP2101 PMIC initialization ---
    if (found_axp) {
        axp2101_init();
    } else {
        Serial.println("[HAL] WARNING: AXP2101 not found! Power rails may not be configured.");
    }

    // --- Step 4: TCA9554 I/O expander — LCD Reset sequence ---
    if (found_tca) {
        Serial.println("[HAL] TCA9554: Performing LCD reset via EXIO1...");
        // Set EXIO1 (LCD_RST) as output
        tca9554_pin_mode(1, false);  // EXIO1 = output
        // Reset pulse: HIGH → LOW → HIGH
        tca9554_write_pin(1, true);   // RST HIGH
        delay(10);
        tca9554_write_pin(1, false);  // RST LOW (active reset)
        delay(10);
        tca9554_write_pin(1, true);   // RST HIGH (release)
        delay(200);                   // Wait for panel to initialize
        Serial.println("[HAL] LCD reset complete");
    } else {
        Serial.println("[HAL] WARNING: TCA9554 not found! LCD reset skipped.");
        Serial.println("[HAL] Display may not initialize correctly without reset.");
    }

    // --- Step 5: Backlight OFF initially ---
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, LCD_BL_ACTIVE_HIGH ? LOW : HIGH);

    // --- Step 6: Initialize QSPI display ---
    Serial.printf("[HAL] Display init: QSPI CS=%d CLK=%d D0=%d D1=%d D2=%d D3=%d\n",
        WS_QSPI_CS, WS_QSPI_SCK, WS_QSPI_D0, WS_QSPI_D1, WS_QSPI_D2, WS_QSPI_D3);
    Serial.println("[HAL] Calling gfx->begin() (Canvas + AXS15231B QSPI)...");

    gfx->begin();
    delay(120);

    // Verify Canvas framebuffer
    if (!gfx->getFramebuffer()) {
        Serial.println("[HAL] FATAL: Canvas framebuffer is NULL! PSRAM not available?");
    } else {
        Serial.printf("[HAL] Canvas framebuffer: %p (PSRAM OK, 8MB available)\n", gfx->getFramebuffer());
    }

    // --- Step 7: Diagnostic pattern ---
    Serial.println("[HAL] Drawing diagnostic quadrants...");
    gfx->fillScreen(BLACK);
    gfx->fillRect(  0,   0, 160, 240, RED);
    gfx->fillRect(160,   0, 160, 240, GREEN);
    gfx->fillRect(  0, 240, 160, 240, BLUE);
    gfx->fillRect(160, 240, 160, 240, WHITE);
    gfx->flush();
    Serial.println("[HAL] Expected landscape: BLUE-RED (top) / WHITE-GREEN (bottom)");

    // --- Step 8: Backlight ON ---
    digitalWrite(PIN_LCD_BL, LCD_BL_ACTIVE_HIGH ? HIGH : LOW);
    gpio_hold_en((gpio_num_t)PIN_LCD_BL);
    Serial.println("[HAL] Backlight ON (GPIO6 locked)");

    delay(500);

    // Clear before LVGL
    gfx->fillScreen(BLACK);
    gfx->flush();
    delay(50);

    // --- Step 9: Touch init ---
    // Touch uses the SAME I2C bus (already initialized in Step 1).
    // AXS15231B touch at 0x3B — same chip/protocol as JC3248W535EN.
    // Reduce I2C clock for touch reads (sensitive to speed).
    Wire.setClock(100000);
    
    if (!found_touch) {
        Serial.println("[HAL] WARNING: AXS15231 touch (0x3B) not found on I2C bus!");
        Serial.println("[HAL] Touch may work anyway — sometimes it only responds after display init.");
        // Retry scan after display init
        Wire.beginTransmission(AXS_TOUCH_ADDR);
        if (Wire.endTransmission() == 0) {
            Serial.println("[HAL] Touch 0x3B found on retry!");
        }
    } else {
        Serial.println("[HAL] Touch initialized (AXS15231B @ 0x3B)");
    }

    Serial.println("[HAL] === Display Init Complete ===");
    Serial.printf("[HAL] Free heap: %u bytes, Free PSRAM: %u bytes\n",
        ESP.getFreeHeap(), ESP.getFreePsram());
    Serial.printf("[HAL] Min free heap: %u bytes\n", ESP.getMinFreeHeap());

    // --- Peripheral Pin Summary ---
    Serial.println("[HAL] === Espresso Peripheral Pin Map ===");
    Serial.printf("[HAL]   TRIAC (AC_PWM):   GPIO%d\n", PIN_AC_PWM);
    Serial.printf("[HAL]   Zero Cross:       GPIO%d\n", PIN_AC_ZC);
    Serial.printf("[HAL]   Solenoid:         GPIO%d\n", PIN_SOLENOID);
    Serial.printf("[HAL]   SSR (heater):     GPIO%d\n", PIN_SSR);
    Serial.printf("[HAL]   MAX31855 CS:      GPIO%d\n", PIN_MAX_CS);
    Serial.printf("[HAL]   MAX31855 DO:      GPIO%d\n", PIN_MAX_DO);
    Serial.printf("[HAL]   MAX31855 CLK:     GPIO%d\n", PIN_MAX_CLK);
    Serial.printf("[HAL]   HX711 DT:         GPIO%d\n", PIN_HX_DT);
    Serial.printf("[HAL]   HX711 SCK:        GPIO%d\n", PIN_HX_SCK);
    Serial.println("[HAL] ================================");
}


uint32_t HAL_ScaleAgeMs() {
    if (last_scale_ok_ms == 0) return 0xFFFFFFFFu;
    return millis() - last_scale_ok_ms;
}

bool HAL_ScaleIsStale() {
    return g_scale_stale;
}
