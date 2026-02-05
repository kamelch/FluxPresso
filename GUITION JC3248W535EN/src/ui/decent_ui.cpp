#include "ui/decent_ui.h"
#include "profile_store.h"
#include "project_config.h"
#include <Arduino.h>
#include <WiFi.h>

extern QueueHandle_t queue_from_ui;

// Helper: Send command to core task
static void send_cmd(UserCommand::Type t, float f = 0.0f, uint32_t u = 0) {
    UserCommand c{};
    c.type = t;
    c.param_f = f;
    c.param_u32 = u;
    c.profile_id = (int)u;
    
    if (xQueueSend(queue_from_ui, &c, pdMS_TO_TICKS(50)) != pdTRUE) {
        Serial.println("ERROR: Command queue full!");
    }
}


// Optional haptic feedback (requires a vibration motor + driver transistor on PIN_HAPTIC)
#ifndef ENABLE_HAPTIC
#define ENABLE_HAPTIC 0
#endif
#ifndef PIN_HAPTIC
#define PIN_HAPTIC -1
#endif

static inline void haptic_pulse(uint16_t ms = 20) {
#if ENABLE_HAPTIC
    if (PIN_HAPTIC >= 0) {
        pinMode((uint8_t)PIN_HAPTIC, OUTPUT);
        digitalWrite((uint8_t)PIN_HAPTIC, HIGH);
        delay(ms);
        digitalWrite((uint8_t)PIN_HAPTIC, LOW);
    }
#else
    (void)ms;
#endif
}


// === COLOR THEME (Dark espresso-inspired) ===
#define COLOR_PRIMARY       0x6366F1  // Indigo
#define COLOR_SECONDARY     0xEC4899  // Pink
#define COLOR_SUCCESS       0x10B981  // Emerald
#define COLOR_WARNING       0xF59E0B  // Amber
#define COLOR_DANGER        0xEF4444  // Red
#define COLOR_INFO          0x3B82F6  // Blue

#define COLOR_BG_GRADIENT_START  0x0F172A
#define COLOR_BG_GRADIENT_END    0x1E293B
#define COLOR_GLASS_CARD         0x111827  // base for glass cards
#define COLOR_BORDER_SOFT        0x334155
#define COLOR_TEXT_PRIMARY       0xE5E7EB
#define COLOR_TEXT_SECONDARY     0x94A3B8
#define COLOR_TEXT_DIM           0x64748B

// Semantic accents used in UI
#define COLOR_ACCENT_TEMP   COLOR_PRIMARY
#define COLOR_ACCENT_WEIGHT COLOR_INFO
#define COLOR_ACCENT_TIME   COLOR_SECONDARY
#define COLOR_ACCENT_BREW   COLOR_SUCCESS
#define COLOR_ACCENT_STEAM  COLOR_DANGER

// Backwards-compatible aliases
#define COLOR_BG_DARK       COLOR_BG_GRADIENT_START
#define COLOR_BG_CARD       COLOR_GLASS_CARD
#define COLOR_BG_CARD_HOVER 0x1F2937
#define COLOR_BORDER        COLOR_BORDER_SOFT

// === UI OBJECTS ===
static lv_obj_t* screen_main = nullptr;
static lv_obj_t* screen_settings = nullptr;
static lv_obj_t* screen_profiles = nullptr;

// Main screen objects
static lv_obj_t* lbl_temp_value = nullptr;
static lv_obj_t* lbl_weight_value = nullptr;
static lv_obj_t* lbl_timer_value = nullptr;
static lv_obj_t* arc_timer = nullptr;
static lv_obj_t* lbl_state = nullptr;
static lv_obj_t* lbl_pressure_value = nullptr;
static lv_obj_t* lbl_flow_value = nullptr;
static lv_obj_t* card_time = nullptr;
static lv_obj_t* lbl_wifi = nullptr;
static lv_obj_t* lbl_setpoint_value = nullptr;

// Chart
static lv_obj_t* chart = nullptr;
static lv_chart_series_t* ser_temp = nullptr;
static lv_chart_series_t* ser_weight_scaled = nullptr;  // Weight/5 for scale

// Controls
static lv_obj_t* btn_brew = nullptr;
static lv_obj_t* btn_stop = nullptr;
static lv_obj_t* btn_steam = nullptr;
static lv_obj_t* btn_flush = nullptr;
static lv_obj_t* btn_settings = nullptr;
static lv_obj_t* btn_profiles = nullptr;
static lv_obj_t* slider_temp = nullptr;
static lv_obj_t* slider_pump = nullptr;
static lv_obj_t* btn_tare = nullptr;

// Settings screen
static lv_obj_t* slider_brew_temp = nullptr;
static lv_obj_t* slider_steam_temp = nullptr;
static lv_obj_t* lbl_brew_temp_val = nullptr;
static lv_obj_t* lbl_steam_temp_val = nullptr;

// Profile screen
static lv_obj_t* dd_profile = nullptr;

// Profile editor state
static ShotProfile g_edit_profile;
static uint8_t g_g_edit_idx = 0;
static lv_obj_t* g_g_s_temp=nullptr;
static lv_obj_t* g_g_s_pre_ms=nullptr;
static lv_obj_t* g_g_s_pre_pct=nullptr;
static lv_obj_t* g_g_s_bloom_ms=nullptr;
static lv_obj_t* g_g_s_bloom_pct=nullptr;
static lv_obj_t* g_g_s_brew_ms=nullptr;
static lv_obj_t* g_g_s_brew_pct=nullptr;
static lv_obj_t* ta_profile_name = nullptr;
static lv_obj_t* spinbox_pre_time = nullptr;
static lv_obj_t* spinbox_pre_pump = nullptr;
static lv_obj_t* spinbox_bloom_time = nullptr;
static lv_obj_t* spinbox_brew_time = nullptr;
static lv_obj_t* spinbox_brew_pump = nullptr;

// Profile store
static ProfileStore g_profiles;
static uint8_t current_profile_id = 0;

// Last known state
static SystemState last_state = STATE_BOOT;
static float last_temp = 0;
static float last_weight = 0;

// === HELPER FUNCTIONS ===

/**
 * Create a modern card container
 */
static lv_obj_t* create_card(lv_obj_t* parent, int w, int h) {
    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);

    // Glassmorphism: semi-transparent dark surface + soft border + elevation shadow
    lv_obj_set_style_bg_color(card, lv_color_hex(COLOR_GLASS_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_70, 0);
    lv_obj_set_style_bg_grad_color(card, lv_color_hex(COLOR_BG_GRADIENT_END), 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);

    lv_obj_set_style_border_color(card, lv_color_hex(COLOR_BORDER_SOFT), 0);
    lv_obj_set_style_border_opa(card, LV_OPA_40, 0);
    lv_obj_set_style_border_width(card, 1, 0);

    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 14, 0);

    // Elevation
    lv_obj_set_style_shadow_width(card, 18, 0);
    lv_obj_set_style_shadow_color(card, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(card, LV_OPA_30, 0);
    lv_obj_set_style_shadow_ofs_y(card, 6, 0);

    return card;
}

/**
 * Create a KPI display card with accent color
 */
static lv_obj_t* create_kpi_card(lv_obj_t* parent, const char* title, const char* value,
                                  lv_color_t accent, lv_obj_t** out_label) {
    lv_obj_t* card = create_card(parent, 140, 100);
    
    // Top accent strip
    lv_obj_t* accent_bar = lv_obj_create(card);
    lv_obj_set_size(accent_bar, lv_pct(100), 3);
    lv_obj_set_style_bg_color(accent_bar, accent, 0);
    lv_obj_set_style_border_width(accent_bar, 0, 0);
    lv_obj_set_style_radius(accent_bar, 0, 0);
    lv_obj_align(accent_bar, LV_ALIGN_TOP_MID, 0, -12);
    
    // Title
    lv_obj_t* lbl_title = lv_label_create(card);
    lv_label_set_text(lbl_title, title);
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(COLOR_TEXT_SECONDARY), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_12, 0);
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 0);
    
    // Value
    lv_obj_t* lbl_value = lv_label_create(card);
    lv_label_set_text(lbl_value, value);
    lv_obj_set_style_text_font(lbl_value, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lbl_value, lv_color_hex(COLOR_TEXT_PRIMARY), 0);
    lv_obj_align(lbl_value, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    
    if (out_label) *out_label = lbl_value;
    return card;
}

/**
 * Create a modern button with icon support
 */
static lv_obj_t* create_button(lv_obj_t* parent, const char* text, lv_color_t accent) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 86, 54);

    // Fluent-ish button: rounded, elevated, with pressed scale
    lv_obj_set_style_radius(btn, 14, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_GLASS_CARD), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(btn, LV_OPA_80, LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(btn, accent, 0);
    lv_obj_set_style_border_opa(btn, LV_OPA_70, 0);
    lv_obj_set_style_border_width(btn, 2, 0);

    lv_obj_set_style_shadow_width(btn, 16, LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(btn, LV_OPA_30, LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(btn, 5, LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(btn, accent, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, LV_OPA_70, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(btn, 8, LV_STATE_PRESSED);

    // Label
    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, accent, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_center(label);

    return btn;
}

/**
 * Get state name string
 */
static const char* get_state_name(SystemState state) {
    switch(state) {
        case STATE_BOOT: return "BOOTING...";
        case STATE_IDLE: return "READY";
        case STATE_BREW_PREHEAT: return "PREHEATING";
        case STATE_BREW_PREINFUSE: return "PREINFUSE";
        case STATE_BREW_HOLD: return "BLOOM";
        case STATE_BREW_FLOW: return "BREWING";
        case STATE_BREW_DRAIN: return "DRAIN";
        case STATE_STEAM_WARMUP: return "STEAM WARMUP";
        case STATE_STEAM_READY: return "STEAM READY";
        case STATE_FLUSH: return "FLUSHING";
        case STATE_DESCALE: return "DESCALING";
        case STATE_FAULT: return "FAULT!";
        default: return "UNKNOWN";
    }
}

/**
 * Get fault description
 */
static const char* get_fault_desc(FaultCode fault) {
    switch(fault) {
        case FAULT_TC_DISCONNECTED: return "Thermocouple disconnected";
        case FAULT_OVERTEMP: return "Overtemperature";
        case FAULT_ZC_MISSING: return "Zero-cross missing";
        case FAULT_BOOT_RESET: return "Boot failure";
        case FAULT_SCALE_ERROR: return "Scale error";
        default: return "Unknown fault";
    }
}

// === EVENT HANDLERS ===

static void btn_brew_event_cb(lv_event_t* e) {
    haptic_pulse();
    send_cmd(UserCommand::CMD_START_BREW);
}

static void btn_stop_event_cb(lv_event_t* e) {
    haptic_pulse();
    send_cmd(UserCommand::CMD_STOP);
}

static void btn_steam_event_cb(lv_event_t* e) {
    haptic_pulse();
    if (last_state == STATE_STEAM_READY || last_state == STATE_STEAM_WARMUP) {
        send_cmd(UserCommand::CMD_EXIT_STEAM);
    } else {
        send_cmd(UserCommand::CMD_ENTER_STEAM);
    }
}

static void btn_flush_event_cb(lv_event_t* e) {
    haptic_pulse();
    send_cmd(UserCommand::CMD_FLUSH);
}

static void btn_tare_event_cb(lv_event_t* e) {
    haptic_pulse();
    send_cmd(UserCommand::CMD_TARE_SCALE);
}

static void slider_temp_event_cb(lv_event_t* e) {
    int32_t val = lv_slider_get_value(slider_temp);
    float temp_c = 80.0f + (val / 100.0f) * 25.0f;  // 80-105°C
    send_cmd(UserCommand::CMD_SET_BREW_TEMP, temp_c);
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f°C", temp_c);
    lv_label_set_text(lbl_setpoint_value, buf);
}

static void slider_pump_event_cb(lv_event_t* e) {
    int32_t val = lv_slider_get_value(slider_pump);
    float pump = val / 100.0f;
    send_cmd(UserCommand::CMD_SET_PUMP_MANUAL, pump);
}

static void btn_settings_event_cb(lv_event_t* e) {
    haptic_pulse();
    lv_scr_load_anim(screen_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 220, 0, false);
}

static void btn_profiles_event_cb(lv_event_t* e) {
    haptic_pulse();
    lv_scr_load_anim(screen_profiles, LV_SCR_LOAD_ANIM_MOVE_LEFT, 220, 0, false);
}

static void btn_back_to_main_event_cb(lv_event_t* e) {
    haptic_pulse();
    lv_scr_load_anim(screen_main, LV_SCR_LOAD_ANIM_FADE_ON, 220, 0, false);
}

static void dd_profile_event_cb(lv_event_t* e) {
    (void)e;
    uint16_t sel = lv_dropdown_get_selected(dd_profile);
    current_profile_id = sel;

    // Tell core (two commands supported in firmware variants; keep both harmless)
    send_cmd(UserCommand::CMD_SELECT_PROFILE, 0.0f, sel);

    // Load into editor + update sliders if they exist
    if (g_profiles.load(sel, g_edit_profile)) {
        g_g_edit_idx = (uint8_t)sel;

        if (g_g_s_temp)      lv_slider_set_value(g_g_s_temp, (int)g_edit_profile.brew_temp_c, LV_ANIM_OFF);
        if (g_g_s_pre_ms)    lv_slider_set_value(g_g_s_pre_ms, (int)g_edit_profile.preinfuse_ms, LV_ANIM_OFF);
        if (g_g_s_pre_pct)   lv_slider_set_value(g_g_s_pre_pct, (int)(g_edit_profile.preinfuse_pump*100.0f), LV_ANIM_OFF);
        if (g_g_s_bloom_ms)  lv_slider_set_value(g_g_s_bloom_ms, (int)g_edit_profile.bloom_ms, LV_ANIM_OFF);
        if (g_g_s_bloom_pct) lv_slider_set_value(g_g_s_bloom_pct, (int)(g_edit_profile.bloom_pump*100.0f), LV_ANIM_OFF);
        if (g_g_s_brew_ms)   lv_slider_set_value(g_g_s_brew_ms, (int)g_edit_profile.brew_ms, LV_ANIM_OFF);
        if (g_g_s_brew_pct)  lv_slider_set_value(g_g_s_brew_pct, (int)(g_edit_profile.brew_pump*100.0f), LV_ANIM_OFF);
    }
}

// === SCREEN CREATION ===

/**
 * Create main brewing screen (landscape layout)
 */
static void create_main_screen() {
    screen_main = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_main, lv_color_hex(COLOR_BG_GRADIENT_START), 0);
    lv_obj_set_style_bg_grad_color(screen_main, lv_color_hex(COLOR_BG_GRADIENT_END), 0);
    lv_obj_set_style_bg_grad_dir(screen_main, LV_GRAD_DIR_VER, 0);
    
    // === LEFT COLUMN: KPIs ===
    lv_obj_t* kpi_container = lv_obj_create(screen_main);
    lv_obj_set_size(kpi_container, 160, 300);
    lv_obj_set_pos(kpi_container, 10, 10);
    lv_obj_set_style_bg_opa(kpi_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(kpi_container, 0, 0);
    lv_obj_set_style_pad_all(kpi_container, 0, 0);
    lv_obj_set_flex_flow(kpi_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(kpi_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(kpi_container, 10, 0);
    
    // Temperature KPI
    create_kpi_card(kpi_container, "TEMP", "---°C", lv_color_hex(COLOR_ACCENT_TEMP), &lbl_temp_value);
    
    // Weight KPI
    create_kpi_card(kpi_container, "WEIGHT", "---g", lv_color_hex(COLOR_ACCENT_WEIGHT), &lbl_weight_value);
    
    // Timer KPI
    card_time = create_kpi_card(kpi_container, "TIME", "--:--", lv_color_hex(COLOR_ACCENT_TIME), &lbl_timer_value);

    // Circular progress for shot timing (Material-like)
    // NOTE: no pressure sensor yet; this is time progress only.
    // Circular progress for shot timing (Material-like)
    arc_timer = lv_arc_create(card_time);
    lv_obj_set_size(arc_timer, 92, 92);
    lv_obj_set_style_arc_width(arc_timer, 10, 0);
    lv_obj_set_style_arc_color(arc_timer, lv_color_hex(COLOR_ACCENT_TIME), 0);
    lv_obj_set_style_arc_opa(arc_timer, LV_OPA_70, 0);
    lv_obj_set_style_bg_opa(arc_timer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arc_timer, 0, 0);
    lv_arc_set_rotation(arc_timer, 270);
    lv_arc_set_bg_angles(arc_timer, 0, 360);
    lv_arc_set_range(arc_timer, 0, 100);
    lv_arc_set_value(arc_timer, 0);
    lv_obj_align(arc_timer, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_flag(arc_timer, LV_OBJ_FLAG_CLICKABLE); // keep touch safe

    
    // === CENTER: CHART ===
    lv_obj_t* chart_card = create_card(screen_main, 210, 240);
    lv_obj_set_pos(chart_card, 180, 10);
    
    chart = lv_chart_create(chart_card);
    lv_obj_set_size(chart, 190, 200);
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, CHART_HISTORY_SECONDS * (1000 / CORE_TASK_RATE_MS));
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 180);  // 0-180°C or 0-900g
    lv_chart_set_div_line_count(chart, 6, 10);
    lv_obj_set_style_bg_color(chart, lv_color_hex(COLOR_BG_GRADIENT_START), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_20, 0);
    lv_obj_set_style_border_width(chart, 0, 0);
    
    ser_temp = lv_chart_add_series(chart, lv_color_hex(COLOR_ACCENT_TEMP), LV_CHART_AXIS_PRIMARY_Y);
    ser_weight_scaled = lv_chart_add_series(chart, lv_color_hex(COLOR_ACCENT_WEIGHT), LV_CHART_AXIS_PRIMARY_Y);
    
    lv_obj_t* chart_title = lv_label_create(chart_card);
    lv_label_set_text(chart_title, "TEMP & WEIGHT");
    lv_obj_set_style_text_color(chart_title, lv_color_hex(COLOR_TEXT_SECONDARY), 0);
    lv_obj_set_style_text_font(chart_title, &lv_font_montserrat_12, 0);
    lv_obj_align(chart_title, LV_ALIGN_TOP_LEFT, 0, -8);
    // Mini gauges: PRESSURE + FLOW (show 0 if sensors not available)
    lv_obj_t* gauge_row = lv_obj_create(screen_main);
    lv_obj_set_size(gauge_row, 210, 48);
    lv_obj_set_pos(gauge_row, 180, 258);
    lv_obj_set_style_bg_opa(gauge_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gauge_row, 0, 0);
    lv_obj_set_style_pad_all(gauge_row, 0, 0);
    lv_obj_set_flex_flow(gauge_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(gauge_row, 10, 0);

    // Pressure card
    lv_obj_t* card_p = create_card(gauge_row, 100, 48);
    lv_obj_set_style_pad_all(card_p, 10, 0);
    lv_obj_clear_flag(card_p, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* lbl_p_t = lv_label_create(card_p);
    lv_label_set_text(lbl_p_t, "PRESS");
    lv_obj_set_style_text_font(lbl_p_t, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl_p_t, lv_color_hex(COLOR_TEXT_SECONDARY), 0);
    lv_obj_align(lbl_p_t, LV_ALIGN_LEFT_MID, 0, 0);
    lbl_pressure_value = lv_label_create(card_p);
    lbl_pressure_value = lv_label_create(card_p);
    lv_label_set_text(lbl_pressure_value, "--.-b");
    lv_obj_set_style_text_font(lbl_pressure_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_pressure_value, lv_color_hex(COLOR_TEXT_PRIMARY), 0);
    lv_obj_align(lbl_pressure_value, LV_ALIGN_RIGHT_MID, 0, 0);

    // Flow card
    lv_obj_t* card_f = create_card(gauge_row, 100, 48);
    lv_obj_set_style_pad_all(card_f, 10, 0);
    lv_obj_clear_flag(card_f, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* lbl_f_t = lv_label_create(card_f);
    lv_label_set_text(lbl_f_t, "FLOW");
    lv_obj_set_style_text_font(lbl_f_t, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl_f_t, lv_color_hex(COLOR_TEXT_SECONDARY), 0);
    lv_obj_align(lbl_f_t, LV_ALIGN_LEFT_MID, 0, 0);
    lbl_flow_value = lv_label_create(card_f);
    lbl_flow_value = lv_label_create(card_f);
    lv_label_set_text(lbl_flow_value, "--.-");
    lv_obj_set_style_text_font(lbl_flow_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_flow_value, lv_color_hex(COLOR_TEXT_PRIMARY), 0);
    lv_obj_align(lbl_flow_value, LV_ALIGN_RIGHT_MID, 0, 0);

    // === RIGHT COLUMN: CONTROLS ===
    lv_obj_t* control_container = lv_obj_create(screen_main);
    lv_obj_set_size(control_container, 90, 280);
    lv_obj_set_pos(control_container, 380, 20);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_set_flex_flow(control_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(control_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(control_container, 8, 0);
    
    // Main action buttons
    btn_brew = create_button(control_container, "BREW", lv_color_hex(COLOR_ACCENT_BREW));
    lv_obj_add_event_cb(btn_brew, btn_brew_event_cb, LV_EVENT_CLICKED, NULL);
    
    btn_stop = create_button(control_container, "STOP", lv_color_hex(COLOR_DANGER));
    lv_obj_add_event_cb(btn_stop, btn_stop_event_cb, LV_EVENT_CLICKED, NULL);
    
    btn_steam = create_button(control_container, "STEAM", lv_color_hex(COLOR_ACCENT_STEAM));
    lv_obj_add_event_cb(btn_steam, btn_steam_event_cb, LV_EVENT_CLICKED, NULL);
    
    btn_flush = create_button(control_container, "FLUSH", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_add_event_cb(btn_flush, btn_flush_event_cb, LV_EVENT_CLICKED, NULL);
    
    btn_tare = create_button(control_container, "TARE", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_add_event_cb(btn_tare, btn_tare_event_cb, LV_EVENT_CLICKED, NULL);
    
    // === BOTTOM BAR: STATUS & SLIDERS ===
    lv_obj_t* status_bar = lv_obj_create(screen_main);
    lv_obj_set_size(status_bar, 460, 60);
    lv_obj_set_pos(status_bar, 10, 250);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(COLOR_BG_CARD), 0);
    lv_obj_set_style_border_width(status_bar, 1, 0);
    lv_obj_set_style_border_color(status_bar, lv_color_hex(COLOR_BORDER), 0);
    lv_obj_set_style_radius(status_bar, 8, 0);
    
    // State label
    lbl_state = lv_label_create(status_bar);
    lv_label_set_text(lbl_state, "READY");
    lv_obj_set_style_text_font(lbl_state, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_state, lv_color_hex(COLOR_ACCENT_BREW), 0);
    lv_obj_set_pos(lbl_state, 10, 8);
    
    // WiFi status
    lbl_wifi = lv_label_create(status_bar);
    lv_label_set_text(lbl_wifi, "WiFi: ---");
    lv_obj_set_style_text_font(lbl_wifi, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl_wifi, lv_color_hex(COLOR_TEXT_DIM), 0);
    lv_obj_set_pos(lbl_wifi, 10, 35);
    
    // Temp setpoint slider
    lv_obj_t* lbl_temp_sp = lv_label_create(status_bar);
    lv_label_set_text(lbl_temp_sp, "Setpoint:");
    lv_obj_set_style_text_font(lbl_temp_sp, &lv_font_montserrat_10, 0);
    lv_obj_set_pos(lbl_temp_sp, 150, 8);
    
    slider_temp = lv_slider_create(status_bar);
    lv_obj_set_size(slider_temp, 120, 10);
    lv_obj_set_pos(slider_temp, 220, 10);
    lv_slider_set_range(slider_temp, 0, 100);
    lv_slider_set_value(slider_temp, 52, LV_ANIM_OFF);  // 93°C = 52%
    lv_obj_add_event_cb(slider_temp, slider_temp_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    lbl_setpoint_value = lv_label_create(status_bar);
    lv_label_set_text(lbl_setpoint_value, "93.0°C");
    lv_obj_set_style_text_font(lbl_setpoint_value, &lv_font_montserrat_10, 0);
    lv_obj_set_pos(lbl_setpoint_value, 350, 8);
    
    // Manual pump slider
    lv_obj_t* lbl_pump = lv_label_create(status_bar);
    lv_label_set_text(lbl_pump, "Manual:");
    lv_obj_set_style_text_font(lbl_pump, &lv_font_montserrat_10, 0);
    lv_obj_set_pos(lbl_pump, 150, 35);
    
    slider_pump = lv_slider_create(status_bar);
    lv_obj_set_size(slider_pump, 120, 10);
    lv_obj_set_pos(slider_pump, 220, 37);
    lv_slider_set_range(slider_pump, 0, 100);
    lv_slider_set_value(slider_pump, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_pump, slider_pump_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Navigation buttons (small)
    btn_settings = create_button(status_bar, "SET", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_set_size(btn_settings, 50, 40);
    lv_obj_set_pos(btn_settings, 360, 10);
    lv_obj_add_event_cb(btn_settings, btn_settings_event_cb, LV_EVENT_CLICKED, NULL);
    
    btn_profiles = create_button(status_bar, "PROF", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_set_size(btn_profiles, 50, 40);
    lv_obj_set_pos(btn_profiles, 415, 10);
    lv_obj_add_event_cb(btn_profiles, btn_profiles_event_cb, LV_EVENT_CLICKED, NULL);
}

/**
 * Create settings screen
 */
static void create_settings_screen() {
    screen_settings = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_settings, lv_color_hex(COLOR_BG_GRADIENT_START), 0);
    lv_obj_set_style_bg_grad_color(screen_settings, lv_color_hex(COLOR_BG_GRADIENT_END), 0);
    lv_obj_set_style_bg_grad_dir(screen_settings, LV_GRAD_DIR_VER, 0);
    
    // Title
    lv_obj_t* title = lv_label_create(screen_settings);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_pos(title, 20, 20);
    
    // Back button
    lv_obj_t* btn_back = create_button(screen_settings, "BACK", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_set_pos(btn_back, 380, 20);
    lv_obj_add_event_cb(btn_back, btn_back_to_main_event_cb, LV_EVENT_CLICKED, NULL);
    
    // TODO: Add settings controls
    lv_obj_t* info = lv_label_create(screen_settings);
    lv_label_set_text(info, "Settings panel\n(under construction)");
    lv_obj_center(info);
}

/**
 * Create profiles screen
 */
static void create_profiles_screen() {
    screen_profiles = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_profiles, lv_color_hex(COLOR_BG_GRADIENT_START), 0);
    lv_obj_set_style_bg_grad_color(screen_profiles, lv_color_hex(COLOR_BG_GRADIENT_END), 0);
    lv_obj_set_style_bg_grad_dir(screen_profiles, LV_GRAD_DIR_VER, 0);
    
    // Title
    lv_obj_t* title = lv_label_create(screen_profiles);
    lv_label_set_text(title, "SHOT PROFILES");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_pos(title, 20, 20);
    
    // Back button
    lv_obj_t* btn_back = create_button(screen_profiles, "BACK", lv_color_hex(COLOR_TEXT_SECONDARY));
    lv_obj_set_pos(btn_back, 380, 20);
    lv_obj_add_event_cb(btn_back, btn_back_to_main_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Profile dropdown
    dd_profile = lv_dropdown_create(screen_profiles);
    lv_obj_set_size(dd_profile, 200, 40);
    lv_obj_set_pos(dd_profile, 20, 70);
    lv_dropdown_set_options(dd_profile, "CLASSIC\nSLAYER\nBLOOM\nFAST\nGENTLE\nUSER");
    lv_obj_add_event_cb(dd_profile, dd_profile_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    
    // --- Interactive Profile Editor (touch-friendly, no drag yet) ---
    g_profiles.load(g_g_edit_idx, g_edit_profile);
    
    // Scroll container for controls
    lv_obj_t* editor = lv_obj_create(screen_profiles);
    lv_obj_set_size(editor, 440, 210);
    lv_obj_set_pos(editor, 20, 120);
    lv_obj_set_style_bg_opa(editor, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(editor, 0, 0);
    lv_obj_set_style_pad_all(editor, 0, 0);
    lv_obj_set_flex_flow(editor, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(editor, 10, 0);
    lv_obj_set_scroll_dir(editor, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(editor, LV_SCROLLBAR_MODE_ACTIVE);

    // Helper: labeled slider row
    auto add_slider_row = [&](const char* label, int minv, int maxv, int init,
                              const char* fmt, lv_event_cb_t cb, lv_obj_t** out_slider) {
        lv_obj_t* row = create_card(editor, 440, 56);
        lv_obj_set_style_pad_left(row, 14, 0);
        lv_obj_set_style_pad_right(row, 14, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* l = lv_label_create(row);
        lv_label_set_text(l, label);
        lv_obj_set_style_text_color(l, lv_color_hex(COLOR_TEXT_SECONDARY), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);

        lv_obj_t* v = lv_label_create(row);
        lv_label_set_text_fmt(v, fmt, init);
        lv_obj_set_style_text_color(v, lv_color_hex(COLOR_TEXT_PRIMARY), 0);
        lv_obj_set_style_text_font(v, &lv_font_montserrat_14, 0);
        lv_obj_align(v, LV_ALIGN_RIGHT_MID, 0, 0);

        lv_obj_t* s = lv_slider_create(row);
        lv_obj_set_size(s, 230, 14);
        lv_obj_align(s, LV_ALIGN_CENTER, 25, 0);
        lv_slider_set_range(s, minv, maxv);
        lv_slider_set_value(s, init, LV_ANIM_OFF);
        lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, v);

        // Modern slider look
        lv_obj_set_style_bg_opa(s, LV_OPA_30, 0);
        lv_obj_set_style_bg_color(s, lv_color_hex(COLOR_BORDER_SOFT), 0);
        lv_obj_set_style_radius(s, 10, 0);
        lv_obj_set_style_outline_width(s, 0, 0);

        // LVGL 8.x: style the knob via LV_PART_KNOB (there is no lv_slider_get_knob())
        lv_obj_set_style_bg_color(s, lv_color_hex(COLOR_PRIMARY), LV_PART_KNOB | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(s, LV_OPA_90, LV_PART_KNOB | LV_STATE_DEFAULT);
        lv_obj_set_style_radius(s, 12, LV_PART_KNOB | LV_STATE_DEFAULT);

        if (out_slider) *out_slider = s;
    };

    // Slider callbacks
    auto cb_temp = [](lv_event_t* e){
        lv_obj_t* s = lv_event_get_target(e);
        lv_obj_t* v = (lv_obj_t*)lv_event_get_user_data(e);
        lv_label_set_text_fmt(v, "%d°C", lv_slider_get_value(s));
    };
    auto cb_ms = [](lv_event_t* e){
        lv_obj_t* s = lv_event_get_target(e);
        lv_obj_t* v = (lv_obj_t*)lv_event_get_user_data(e);
        lv_label_set_text_fmt(v, "%dms", lv_slider_get_value(s));
    };
    auto cb_pct = [](lv_event_t* e){
        lv_obj_t* s = lv_event_get_target(e);
        lv_obj_t* v = (lv_obj_t*)lv_event_get_user_data(e);
        lv_label_set_text_fmt(v, "%d%%", lv_slider_get_value(s));
    };

    g_g_s_temp=nullptr;
    g_g_s_pre_ms=nullptr;
    g_g_s_pre_pct=nullptr;
    g_g_s_bloom_ms=nullptr;
    g_g_s_bloom_pct=nullptr;
    g_g_s_brew_ms=nullptr;
    g_g_s_brew_pct=nullptr;

    add_slider_row("Brew Temp", 80, 105, (int)g_edit_profile.brew_temp_c, "%d°C", cb_temp, &g_g_s_temp);
    add_slider_row("Preinfuse Time", 0, 15000, (int)g_edit_profile.preinfuse_ms, "%dms", cb_ms, &g_g_s_pre_ms);
    add_slider_row("Preinfuse Pump", 0, 100, (int)(g_edit_profile.preinfuse_pump*100.0f), "%d%%", cb_pct, &g_g_s_pre_pct);
    add_slider_row("Bloom Time", 0, 15000, (int)g_edit_profile.bloom_ms, "%dms", cb_ms, &g_g_s_bloom_ms);
    add_slider_row("Bloom Pump", 0, 100, (int)(g_edit_profile.bloom_pump*100.0f), "%d%%", cb_pct, &g_g_s_bloom_pct);
    add_slider_row("Brew Time", 5000, 60000, (int)g_edit_profile.brew_ms, "%dms", cb_ms, &g_g_s_brew_ms);
    add_slider_row("Brew Pump", 0, 100, (int)(g_edit_profile.brew_pump*100.0f), "%d%%", cb_pct, &g_g_s_brew_pct);

    // Save / Activate
    lv_obj_t* btn_save = create_button(screen_profiles, "SAVE", lv_color_hex(COLOR_SUCCESS));
    lv_obj_set_pos(btn_save, 260, 70);
    lv_obj_add_event_cb(btn_save, [](lv_event_t*){
        haptic_pulse();
        g_edit_profile.brew_temp_c    = (float)lv_slider_get_value(g_g_s_temp);
        g_edit_profile.preinfuse_ms   = (uint32_t)lv_slider_get_value(g_g_s_pre_ms);
        g_edit_profile.preinfuse_pump = (float)lv_slider_get_value(g_g_s_pre_pct) / 100.0f;
        g_edit_profile.bloom_ms       = (uint32_t)lv_slider_get_value(g_g_s_bloom_ms);
        g_edit_profile.bloom_pump     = (float)lv_slider_get_value(g_g_s_bloom_pct) / 100.0f;
        g_edit_profile.brew_ms        = (uint32_t)lv_slider_get_value(g_g_s_brew_ms);
        g_edit_profile.brew_pump      = (float)lv_slider_get_value(g_g_s_brew_pct) / 100.0f;

        g_profiles.save(g_g_edit_idx, g_edit_profile);
        g_profiles.setActive(g_g_edit_idx);
        Serial.println("Profile saved + activated");
    }, LV_EVENT_CLICKED, NULL);

    // NOTE: A full drag-and-drop curve editor requires a custom widget; this editor is the
    // most reliable touch UI on LVGL 8 with no extra dependencies.

}

// === PUBLIC FUNCTIONS ===

void UI_Init() {
    Serial.println("Initializing UI...");
    
    // Apply dark theme
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(COLOR_BG_DARK), 0);
    
    // Initialize profile store
    g_profiles.begin();
    g_profiles.ensureDefaults();
    
    // Create screens
    create_main_screen();
    create_settings_screen();
    create_profiles_screen();
    
    // Load main screen
    lv_scr_load_anim(screen_main, LV_SCR_LOAD_ANIM_FADE_ON, 220, 0, false);
    
    Serial.println("UI initialized");
}

void UI_Update(const StateSnapshot& snap) {
    // Update KPIs
    char buf[32];
    
    // Temperature
    snprintf(buf, sizeof(buf), "%.1f°C", snap.temp_c);
    lv_label_set_text(lbl_temp_value, buf);
    
    // Weight
    snprintf(buf, sizeof(buf), "%.1fg", snap.weight_g);
    lv_label_set_text(lbl_weight_value, buf);

    // Pressure + Flow (if sensors not implemented, values may remain 0.0)
    if (lbl_pressure_value) {
        snprintf(buf, sizeof(buf), "%.1fb", snap.pressure_bar);
        lv_label_set_text(lbl_pressure_value, buf);
    }
    if (lbl_flow_value) {
        if (snap.flow_rate > 0.01f) {
            snprintf(buf, sizeof(buf), "%.1f", snap.flow_rate);
        } else {
            // No flow sensor yet: show pump manual as a proxy
            snprintf(buf, sizeof(buf), "P%u%%", (unsigned)(snap.pump_manual * 100.0f));
        }
        lv_label_set_text(lbl_flow_value, buf);
    }

// // Timer progress ring
    if (arc_timer) {
        if (snap.state >= STATE_BREW_PREINFUSE && snap.state <= STATE_BREW_DRAIN) {
            const uint32_t cap_ms = 30000; // soft cap (30s). Later: use profile duration.
            uint32_t vms = snap.shot_ms;
            if (vms > cap_ms) vms = cap_ms;
            uint16_t pct = (uint16_t)((vms * 100) / cap_ms);
            lv_arc_set_value(arc_timer, pct);
        } else {
            lv_arc_set_value(arc_timer, 0);
        }
    }

// State
    lv_label_set_text(lbl_state, get_state_name(snap.state));
    
    // Change state color based on state
    if (snap.state == STATE_FAULT) {
        lv_obj_set_style_text_color(lbl_state, lv_color_hex(COLOR_DANGER), 0);
        lv_label_set_text(lbl_state, get_fault_desc(snap.fault));
    } else if (snap.state >= STATE_BREW_PREINFUSE && snap.state <= STATE_BREW_DRAIN) {
        lv_obj_set_style_text_color(lbl_state, lv_color_hex(COLOR_ACCENT_BREW), 0);
    } else if (snap.state == STATE_STEAM_READY || snap.state == STATE_STEAM_WARMUP) {
        lv_obj_set_style_text_color(lbl_state, lv_color_hex(COLOR_ACCENT_STEAM), 0);
    } else {
        lv_obj_set_style_text_color(lbl_state, lv_color_hex(COLOR_TEXT_PRIMARY), 0);
    }
    
    // WiFi status
    if (snap.wifi_connected) {
        snprintf(buf, sizeof(buf), "WiFi: %ddBm", snap.wifi_rssi);
    } else {
        strcpy(buf, "WiFi: ---");
    }
    lv_label_set_text(lbl_wifi, buf);
    
    // Update chart
    lv_chart_set_next_value(chart, ser_temp, (int32_t)snap.temp_c);
    lv_chart_set_next_value(chart, ser_weight_scaled, (int32_t)(snap.weight_g / 5.0f));  // Scale for visibility
    
    // Update button states
    bool brewing = (snap.state >= STATE_BREW_PREHEAT && snap.state <= STATE_BREW_DRAIN);
    if (brewing) {
        lv_obj_add_state(btn_brew, LV_STATE_DISABLED);
    } else {
        lv_obj_clear_state(btn_brew, LV_STATE_DISABLED);
    }
    
    // Cache state
    last_state = snap.state;
    last_temp = snap.temp_c;
    last_weight = snap.weight_g;
}

namespace UI {
    lv_obj_t* get_screen() {
        return screen_main;
    }
}