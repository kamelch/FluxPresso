#include <Arduino.h>
#include "project_config.h"
#include "hal.h"
#include "common_types.h"

// Task declarations
extern void core_task(void *pvParameters);
extern void ui_task(void *pvParameters);
extern void network_task(void *pvParameters);

// Inter-task communication queues
QueueHandle_t queue_to_ui;
QueueHandle_t queue_from_ui;
QueueHandle_t queue_to_net;

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n\n========================================");
    Serial.println("Enhanced Espresso Controller v2.0");
    Serial.println("with PID & DreamSteam");
    Serial.println("========================================\n");

    // Initialize display and LVGL
    HAL_Display_Init();
    
    // Create FreeRTOS queues
    queue_to_ui = xQueueCreate(10, sizeof(StateSnapshot));
    queue_from_ui = xQueueCreate(5, sizeof(UserCommand));
    queue_to_net = xQueueCreate(5, sizeof(NetCommand));

    if (!queue_to_ui || !queue_from_ui || !queue_to_net) {
        Serial.println("FATAL: Queue creation failed!");
        while(1) delay(1000);
    }

    // Create tasks
    // Core task: Highest priority - controls hardware
    xTaskCreatePinnedToCore(
        core_task,
        "CoreTask",
        4096,
        NULL,
        3,  // High priority
        NULL,
        1   // Core 1
    );

    // UI task: Medium priority
    xTaskCreatePinnedToCore(
        ui_task,
        "UITask",
        8192,
        NULL,
        2,  // Medium priority
        NULL,
        0   // Core 0 (same as Arduino loop)
    );

    // Network task: Lowest priority
    xTaskCreatePinnedToCore(
        network_task,
        "NetworkTask",
        4096,
        NULL,
        1,  // Low priority
        NULL,
        0   // Core 0
    );

    Serial.println("Tasks created successfully");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
}

void loop() {
    // Arduino loop runs on Core 0
    // FreeRTOS tasks handle everything, so just delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Optional: Print system stats
    static uint32_t last_stats_ms = 0;
    if (millis() - last_stats_ms > 10000) {
        Serial.printf("Uptime: %lu s, Free heap: %d bytes\n", 
                     millis() / 1000, ESP.getFreeHeap());
        last_stats_ms = millis();
    }
}
