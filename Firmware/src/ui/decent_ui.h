#pragma once
#include <lvgl.h>
#include "common_types.h"

// Initialize UI
void UI_Init(void);

// Update UI with new state
void UI_Update(const StateSnapshot& snap);

// Get UI objects for direct access if needed
namespace UI {
    lv_obj_t* get_screen();
}
