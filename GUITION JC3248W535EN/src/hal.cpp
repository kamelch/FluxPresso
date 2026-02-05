#include "hal.h"
#include "project_config.h"

#include <TAMC_GT911.h>
#include <Arduino_GFX_Library.h>
// These specific includes are required for GFX v1.5.7
#include <databus/Arduino_ESP32QSPI.h>
// UPDATE: Corrected to use the 'B' variant header which exists in this version
#include <display/Arduino_AXS15231B.h> 

#include <Adafruit_MAX31855.h>
#include "HX711.h"

// --- DISPLAY DRIVER (AXS15231B QSPI) ---
// Pins specific to JC3248W535EN / Guition S3 3.5"
#define GFX_QSPI_CS  45
#define GFX_QSPI_SCK 47
#define GFX_QSPI_D0  21
#define GFX_QSPI_D1  48
#define GFX_QSPI_D2  40
#define GFX_QSPI_D3  39
#define GFX_RST      -1 

// Touch Pins (GT911 I2C)
#define TOUCH_SDA    4
#define TOUCH_SCL    8
#define TOUCH_RST    -1
#define TOUCH_IRQ    -1 

// GT911 touch controller (I2C). RST/IRQ may be -1 if not wired.
// Landscape mode - rotation 0 (no rotation)
static constexpr int LCD_ROTATION = 0;
static TAMC_GT911 touch(TOUCH_SDA, TOUCH_SCL, TOUCH_IRQ, TOUCH_RST, LCD_WIDTH, LCD_HEIGHT);

// Initialize QSPI Data Bus
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    GFX_QSPI_CS, GFX_QSPI_SCK, GFX_QSPI_D0, GFX_QSPI_D1, GFX_QSPI_D2, GFX_QSPI_D3
);

// Initialize Display Driver (AXS15231B)
// UPDATE: Corrected class name to Arduino_AXS15231B
Arduino_GFX *gfx = new Arduino_AXS15231B(
    bus, GFX_RST, LCD_ROTATION /* rotation */, false /* IPS */, LCD_HEIGHT, LCD_WIDTH
);

// --- SENSORS & ACTUATORS ---
Adafruit_MAX31855 thermocouple(PIN_MAX_CLK, PIN_MAX_CS, PIN_MAX_DO);
HX711 scale;

// --- DIMMER VARIABLES ---
hw_timer_t *dimmerTimer = NULL;
volatile uint32_t dimming_delay_us = 0;
volatile bool pump_active = false;

// ZC health tracking (ISR-safe): count pulses in ISR, evaluate in task.
static volatile uint32_t zc_count = 0;
static uint32_t last_zc_count = 0;
static uint32_t last_zc_check_ms = 0;

// Slow PWM (time proportioning) for SSR heater.
static uint32_t heater_pwm_epoch_ms = 0;

// Scale timeout tracking
static uint32_t last_scale_ok_ms = 0;

// --- ISRs ---
void IRAM_ATTR HAL_ISR_DimmerTimer() {
    digitalWrite(PIN_AC_PWM, HIGH);
    ets_delay_us(TRIAC_PULSE_US);
    digitalWrite(PIN_AC_PWM, LOW);
}

void IRAM_ATTR HAL_ISR_ZeroCross() {
    // millis() is NOT ISR-safe. Use a counter.
    zc_count++;
    if (pump_active && dimming_delay_us > 0) {
        timerAlarmWrite(dimmerTimer, dimming_delay_us, false);
        timerAlarmEnable(dimmerTimer);
    }
}

// --- INIT ---
bool HAL_Init() {
    bool ok = true;

    // 1. Outputs
    pinMode(PIN_SOLENOID, OUTPUT); digitalWrite(PIN_SOLENOID, LOW);
    pinMode(PIN_SSR, OUTPUT);      digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_AC_PWM, OUTPUT);   digitalWrite(PIN_AC_PWM, LOW);
    
    // 2. Inputs
    pinMode(PIN_AC_ZC, INPUT_PULLUP);
    
    // 3. Sensors
    if (!thermocouple.begin()) {
        ok = false;
    }
    scale.begin(PIN_HX_DT, PIN_HX_SCK);

    heater_pwm_epoch_ms = millis();
    last_scale_ok_ms = millis();

    // 4. Dimmer Timer (Timer 0, 80MHz/80 = 1us tick)
    // Using Core 2.x standard syntax
    dimmerTimer = timerBegin(0, 80, true);
    if (dimmerTimer == NULL) {
        ok = false;
    } else {
        timerAttachInterrupt(dimmerTimer, &HAL_ISR_DimmerTimer, true);
    }
    attachInterrupt(digitalPinToInterrupt(PIN_AC_ZC), HAL_ISR_ZeroCross, RISING);

    // Quick sanity read
    double c = thermocouple.readCelsius();
    if (isnan(c)) {
        ok = false;
    }
    return ok;
}

void HAL_ReadSensors(float &temp, float &weight, bool &zc_health) {
    // Temp
    double c = thermocouple.readCelsius();
    temp = isnan(c) ? 999.0 : c;

    // Weight
    if (scale.is_ready()) {
        float new_w = scale.get_units(1);
        // Basic sanity window. Reject absurd values.
        if (new_w > -500.0f && new_w < 500.0f) {
            weight = new_w;
            last_scale_ok_ms = millis();
        }
    }

    // Scale timeout -> sentinel value (UI can show "--" and core can ignore).
    if ((millis() - last_scale_ok_ms) > 5000) {
        weight = -999.0f;
    }

    // ZC health check (evaluate pulse counter every 100ms)
    const uint32_t now = millis();
    if ((now - last_zc_check_ms) >= 100) {
        const uint32_t ccount = zc_count;
        zc_health = (ccount != last_zc_count);
        last_zc_count = ccount;
        last_zc_check_ms = now;
    }
}

void HAL_TareScale() {
    scale.tare();
}

void HAL_SetOutputs(uint8_t pump_percent, bool solenoid_open, uint8_t heater_duty) {
    // Input validation
    if (pump_percent > 100) pump_percent = 100;
    if (heater_duty > 100) heater_duty = 100;

    digitalWrite(PIN_SOLENOID, solenoid_open ? HIGH : LOW);

    // Slow PWM (time proportioning) for SSR. This makes heater_duty meaningful.
    // 1s period avoids SSR chattering.
    const uint32_t HEATER_PERIOD_MS = 1000;
    const uint32_t now = millis();
    const uint32_t phase = (now - heater_pwm_epoch_ms) % HEATER_PERIOD_MS;
    const uint32_t on_ms = (uint32_t)heater_duty * HEATER_PERIOD_MS / 100;
    digitalWrite(PIN_SSR, (heater_duty > 0 && phase < on_ms) ? HIGH : LOW);

    // Pump Dimmer Calculation
    if (pump_percent == 0) {
        pump_active = false;
        if (dimmerTimer) timerAlarmDisable(dimmerTimer);
        digitalWrite(PIN_AC_PWM, LOW);
    } else {
        if (!dimmerTimer) {
            // Fail-safe if timer isn't available.
            pump_active = false;
            digitalWrite(PIN_AC_PWM, LOW);
            return;
        }
        pump_active = true;
        // 50Hz half-cycle = 10,000us
        const int HALF_WAVE_US = 10000;
        const int MIN_DELAY_US = 500;   // max power
        const int MAX_DELAY_US = 9000;  // min power
        // Correct linear mapping: higher percent -> shorter delay
        int delay = MIN_DELAY_US + ((100 - (int)pump_percent) * (MAX_DELAY_US - MIN_DELAY_US) / 100);
        if (delay < MIN_DELAY_US) delay = MIN_DELAY_US;
        if (delay > MAX_DELAY_US) delay = MAX_DELAY_US;
        dimming_delay_us = (uint32_t)delay;
    }
}

// --- DISPLAY ADAPTER ---
void HAL_Display_Flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t*)&color_p->full, w, h);
}

void HAL_Touch_Read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    (void)indev_driver;

    // Read touch controller (GT911). Non-blocking.
    touch.read();

    if (touch.isTouched) {
        uint16_t tx = (uint16_t)touch.points[0].x;
        uint16_t ty = (uint16_t)touch.points[0].y;

        // Map to LVGL's portrait coordinate system (LCD_WIDTH x LCD_HEIGHT)
        uint16_t x = tx;
        uint16_t y = ty;

        switch (LCD_ROTATION & 3) {
            case 0: // 0°
                x = tx; y = ty;
                break;
            case 1: // 90°
                x = ty;
                y = (LCD_HEIGHT - 1) - tx;
                break;
            case 2: // 180°
                x = (LCD_WIDTH - 1) - tx;
                y = (LCD_HEIGHT - 1) - ty;
                break;
            case 3: // 270°
                x = (LCD_WIDTH - 1) - ty;
                y = tx;
                break;
        }

        // Clamp
        if (x >= LCD_WIDTH)  x = LCD_WIDTH - 1;
        if (y >= LCD_HEIGHT) y = LCD_HEIGHT - 1;

        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}


void HAL_Display_Init() {
    // Init Arduino_GFX
    gfx->begin();

    // Touch init (GT911 over I2C)
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setClock(400000);
    touch.begin();
    gfx->fillScreen(BLACK);
}