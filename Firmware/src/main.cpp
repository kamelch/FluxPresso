#include <Arduino.h>
#include "project_config.h"
#include "hal.h"
#include "common_types.h"
#include "esp_task_wdt.h"
#include "tasks/soak_task.h"

// Task declarations
extern void core_task(void *pvParameters);
extern void ui_task(void *pvParameters);
extern void network_task(void *pvParameters);

// Inter-task communication queues
QueueHandle_t queue_to_ui;
QueueHandle_t queue_from_ui;
QueueHandle_t queue_to_net;

void setup() {
    // BOOT-TIME OUTPUT SAFETY: force critical outputs OFF before any init that may stall.
    HAL_ForceOutputsSafeEarly();

    Serial.begin(115200);
    // ESP32-S3 USB-CDC needs time to enumerate
    delay(2000);
    
    Serial.println("\n\n========================================");
    Serial.println("Enhanced Espresso Controller v2.0");
    Serial.println("with PID & DreamSteam");
    Serial.println("========================================\n");

    // Log reset reason
    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("Reset reason: %d ", (int)rr);
    switch (rr) {
        case ESP_RST_POWERON:  Serial.println("(POWERON)"); break;
        case ESP_RST_SW:       Serial.println("(SOFTWARE)"); break;
        case ESP_RST_PANIC:    Serial.println("(PANIC)"); break;
        case ESP_RST_INT_WDT:  Serial.println("(INTERRUPT WDT)"); break;
        case ESP_RST_TASK_WDT: Serial.println("(TASK WDT)"); break;
        case ESP_RST_WDT:      Serial.println("(OTHER WDT)"); break;
        default:               Serial.println("(OTHER)"); break;
    }

    Serial.printf("Free heap: %u, PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());

    // *** WATCHDOG DISABLED FOR DEBUGGING ***
    // This was causing rst:0xc (RTC_SW_CPU_RST) reboot loops.
    // DO NOT enable until display is confirmed stable.
    // esp_task_wdt_init(10, true);
    Serial.println("*** WDT DISABLED FOR DEBUG ***");

    // Initialize display
    HAL_Display_Init();

    Serial.printf("Post-display heap: %u, PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());

    // Create FreeRTOS queues
    queue_to_ui = xQueueCreate(1, sizeof(StateSnapshot));
    queue_from_ui = xQueueCreate(5, sizeof(UserCommand));
    queue_to_net = xQueueCreate(5, sizeof(NetCommand));

    if (!queue_to_ui || !queue_from_ui || !queue_to_net) {
        Serial.println("FATAL: Queue creation failed!");
        while(1) delay(1000);
    }

    // Core task: Highest priority - controls hardware
    xTaskCreatePinnedToCore(
        core_task, "CoreTask", 8192, NULL,
        3, NULL, 1  // Core 1
    );

    // UI task (16KB stack for LVGL + Canvas)
    xTaskCreatePinnedToCore(
        ui_task, "UITask", 16384, NULL,
        2, NULL, 0  // Core 0
    );

    // Network task
    xTaskCreatePinnedToCore(
        network_task, "NetworkTask", 4096, NULL,
        1, NULL, 0  // Core 0
    );

    Serial.println("Tasks created successfully");
    Serial.printf("Free heap: %u\n", ESP.getFreeHeap());

    start_soak_task();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    static uint32_t last_stats_ms = 0;
    if (millis() - last_stats_ms > 5000) {
        Serial.printf("[MAIN] Uptime: %lus, Heap: %u, MinHeap: %u, PSRAM: %u\n", 
                     millis() / 1000, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
        last_stats_ms = millis();
    }
}
