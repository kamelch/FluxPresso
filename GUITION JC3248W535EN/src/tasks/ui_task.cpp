#include <Arduino.h>
#include <lvgl.h>
#include "common_types.h"
#include "hal.h"
#include "ui/decent_ui.h"
#include "project_config.h"

extern QueueHandle_t queue_to_ui;

// LVGL display and input buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LCD_WIDTH * 40];  // 40 lines buffer
static lv_color_t buf2[LCD_WIDTH * 40];  // Double buffering

/**
 * LVGL display flush callback
 */
static void disp_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    // FIX: Pass pointers directly to HAL, as declared in hal.h
    HAL_Display_Flush(disp, area, color_p);
    lv_disp_flush_ready(disp);
}

/**
 * LVGL touch read callback
 */
static void touchpad_read_cb(lv_indev_drv_t* indev_driver, lv_indev_data_t* data) {
    // FIX: Pass driver and data pointers directly to HAL to be filled
    HAL_Touch_Read(indev_driver, data);
}

/**
 * Initialize LVGL
 */
static void init_lvgl() {
    Serial.println("Initializing LVGL...");
    
    lv_init();
    
    // Setup display buffer (double buffering)
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * 40);
    
    // Register display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = disp_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    // Register touch input driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read_cb;
    lv_indev_drv_register(&indev_drv);
    
    Serial.println("LVGL initialized");
}

/**
 * UI Task - handles LVGL updates and user input
 */
void ui_task(void *pvParameters) {
    Serial.println("UI task started");
    
    // Initialize LVGL
    init_lvgl();
    
    // Create UI screens
    UI_Init();
    
    Serial.println("UI ready");
    
    TickType_t lastWake = xTaskGetTickCount();
    StateSnapshot snap{};
    
    // Main UI loop - runs at 20Hz (50ms)
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_TASK_RATE_MS));
        
        // Check for state updates from core task
        while (xQueueReceive(queue_to_ui, &snap, 0) == pdTRUE) {
            UI_Update(snap);
        }
        
        // Let LVGL handle timers and animations
        lv_timer_handler();
    }
}