#include <Arduino.h>
#include "../diagnostics.h"
#include "../hal.h"
#include "esp_task_wdt.h"

// Simple on-device soak harness (disabled by default).
// Enable with build flag: -DENABLE_SOAK_TEST=1
#ifndef ENABLE_SOAK_TEST
#define ENABLE_SOAK_TEST 0
#endif

#if ENABLE_SOAK_TEST

static uint32_t g_last_ms = 0;
static uint32_t g_max_sensor_us = 0;
static uint32_t g_min_sensor_us = 0xFFFFFFFF;

static void soak_task(void* pv) {
    (void)pv;
    Diag::log(LOG_WARN, "SOAK: started (heap/jitter/wdt/sensor-read)");

    for (;;) {
        const uint32_t t0 = micros();
        float temp=0, weight=0; bool zc_ok=false;
        HAL_ReadSensors(temp, weight, zc_ok);
        const uint32_t dt = (uint32_t)(micros() - t0);
        if (dt > g_max_sensor_us) g_max_sensor_us = dt;
        if (dt < g_min_sensor_us) g_min_sensor_us = dt;

        // Every 10s, emit a compact health line.
        const uint32_t now = millis();
        if (g_last_ms == 0) g_last_ms = now;
        if (now - g_last_ms >= 10000) {
            Diag::logf(LOG_INFO, "SOAK: heap=%u min_read_us=%lu max_read_us=%lu zc_ok=%d temp=%.1f",
                       (unsigned)ESP.getFreeHeap(),
                       (unsigned long)g_min_sensor_us,
                       (unsigned long)g_max_sensor_us,
                       zc_ok ? 1 : 0,
                       temp);
            g_last_ms = now;
            g_max_sensor_us = 0;
            g_min_sensor_us = 0xFFFFFFFF;
        }

        // Feed WDT for this task if enabled globally.
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void start_soak_task() {
    xTaskCreatePinnedToCore(soak_task, "SoakTask", 3072, NULL, 1, NULL, 0);
}

#else

void start_soak_task() {}

#endif
