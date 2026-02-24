#include <Arduino.h>
#include <lvgl.h>
#include "common_types.h"
#include "hal.h"
#include "ui/decent_ui.h"
#include "project_config.h"

extern QueueHandle_t queue_to_ui;

// Single LVGL buffer in internal SRAM (40 lines × 480px × 2 bytes = 38.4KB)
// Canvas holds the full frame in PSRAM anyway, so double-buffering is unnecessary.
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LCD_WIDTH * 40];

static void disp_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    HAL_Display_Flush(disp, area, color_p);
}

static void touchpad_read_cb(lv_indev_drv_t* indev_driver, lv_indev_data_t* data) {
    HAL_Touch_Read(indev_driver, data);
}

static void init_lvgl() {
    Serial.println("[UI] Initializing LVGL...");
    
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_WIDTH * 40);
    
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = disp_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read_cb;
    lv_indev_drv_register(&indev_drv);
    
    Serial.printf("[UI] LVGL initialized. Heap: %u\n", ESP.getFreeHeap());
}

void ui_task(void *pvParameters) {
    // NO WDT registration — disabled for debugging
    Serial.println("[UI] Task started");
    
    init_lvgl();
    
    Serial.println("[UI] Creating UI screens...");
    UI_Init();
    Serial.printf("[UI] UI ready. Heap: %u, Stack HWM: %u\n",
        ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(NULL));
    
    TickType_t lastWake = xTaskGetTickCount();
    StateSnapshot snap{};
    uint32_t frame_count = 0;
    
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_TASK_RATE_MS));
        
        while (xQueueReceive(queue_to_ui, &snap, 0) == pdTRUE) {
            UI_Update(snap);
        }
        
        lv_timer_handler();
        
        frame_count++;
        if (frame_count <= 10 || (frame_count % 200) == 0) {
            Serial.printf("[UI] Frame %u, heap: %u, stack: %u\n",
                frame_count, ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(NULL));
        }
    }
}
