// DASHBOARD_UI_SLAYER_V1
// Decent/Gaggiuino-like dashboard adapted for 3.5" 480x320 (landscape)
#include "ui/decent_ui.h"
#include "common_types.h"
#include <Arduino.h>
#include <math.h>

extern QueueHandle_t queue_from_ui;

// ----------------------------
// Command helper
// ----------------------------
static void send_cmd(UserCommand::Type t, float f = 0.0f, uint32_t u = 0) {
    UserCommand c{};
    c.type = t;
    c.param_f = f;
    c.param_u32 = u;
    c.profile_id = (int)u;
    (void)xQueueSend(queue_from_ui, &c, pdMS_TO_TICKS(20));
}

// ----------------------------
// UI objects
// ----------------------------
static lv_obj_t* scr_main = nullptr;
static lv_obj_t* scr_settings = nullptr;

// Header
static lv_obj_t* lbl_title = nullptr;
static lv_obj_t* lbl_sub   = nullptr;
static lv_obj_t* lbl_wifi  = nullptr;
static lv_obj_t* btn_settings = nullptr;
static lv_obj_t* btn_power    = nullptr;

// Cards
static lv_obj_t* card_temp = nullptr;
static lv_obj_t* card_press = nullptr;
static lv_obj_t* card_time = nullptr;

static lv_obj_t* lbl_temp_value = nullptr;
static lv_obj_t* lbl_temp_target = nullptr;
static lv_obj_t* pill_temp = nullptr;
static lv_obj_t* lbl_pill_temp = nullptr;

static lv_obj_t* lbl_press_value = nullptr;
static lv_obj_t* bar_press = nullptr;
static lv_obj_t* btn_tare_small = nullptr;

static lv_obj_t* lbl_time_value = nullptr;
static lv_obj_t* lbl_weight_line = nullptr;

// Actions
static lv_obj_t* btn_brewstop = nullptr;
static lv_obj_t* lbl_brewstop = nullptr;

static lv_obj_t* btn_steam = nullptr;
static lv_obj_t* btn_hot = nullptr;
static lv_obj_t* btn_clean = nullptr;

// Settings screen widgets
static lv_obj_t* lbl_set_temp = nullptr;
static lv_obj_t* lbl_set_scale = nullptr;
static float g_brew_sp = 93.0f;
static float g_scale_fac = 0.0f;

// Last snapshot for button logic
static SystemState g_state = STATE_BOOT;
static FaultCode   g_fault = FAULT_NONE;
static bool        g_wifi_ok = false;

// STOP repeat timer for "hold to reset"
static lv_timer_t* g_stop_repeat_timer = nullptr;
static bool        g_stop_pressed = false;

// ----------------------------
// Styling helpers
// ----------------------------
static void style_root(lv_obj_t* scr) {
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xF3F4F6), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
}

static lv_obj_t* make_card(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* c = lv_obj_create(parent);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_size(c, w, h);
    lv_obj_set_style_radius(c, 14, 0);
    lv_obj_set_style_bg_color(c, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_border_color(c, lv_color_hex(0xE5E7EB), 0);
    lv_obj_set_style_shadow_width(c, 16, 0);
    lv_obj_set_style_shadow_opa(c, LV_OPA_20, 0);
    lv_obj_set_style_shadow_ofs_y(c, 6, 0);
    lv_obj_set_style_pad_all(c, 12, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

static lv_obj_t* make_small_btn(lv_obj_t* parent, int w, int h, const char* txt) {
    lv_obj_t* b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_radius(b, 12, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(0xE5E7EB), 0);
    lv_obj_set_style_shadow_width(b, 14, 0);
    lv_obj_set_style_shadow_opa(b, LV_OPA_10, 0);
    lv_obj_set_style_shadow_ofs_y(b, 5, 0);

    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0x111827), 0);
    lv_obj_center(l);
    return b;
}

static void set_btn_enabled(lv_obj_t* b, bool en) {
    if (!b) return;
    if (en) {
        lv_obj_clear_state(b, LV_STATE_DISABLED);
        lv_obj_set_style_opa(b, LV_OPA_COVER, 0);
    } else {
        lv_obj_add_state(b, LV_STATE_DISABLED);
        lv_obj_set_style_opa(b, LV_OPA_50, 0);
    }
}

static const char* state_text(SystemState s) {
    switch (s) {
        case STATE_IDLE: return "Ready";
        case STATE_BREW_PREHEAT: return "Heating";
        case STATE_BREW_PREINFUSE: return "Pre-brew";
        case STATE_BREW_HOLD: return "Bloom";
        case STATE_BREW_FLOW: return "Brew";
        case STATE_BREW_DRAIN: return "Drain";
        case STATE_STEAM_WARMUP: return "Steam";
        case STATE_STEAM_READY: return "Steam Ready";
        case STATE_FLUSH: return "Hot Water";
        case STATE_FAULT: return "Fault";
        default: return "Run";
    }
}

// ----------------------------
// Callbacks
// ----------------------------
static void stop_repeat_cb(lv_timer_t* t) {
    (void)t;
    if (!g_stop_pressed) return;
    send_cmd(UserCommand::CMD_STOP);
}

static void brewstop_event(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        if (g_state == STATE_IDLE && g_fault == FAULT_NONE) {
            send_cmd(UserCommand::CMD_START_BREW);
        } else {
            send_cmd(UserCommand::CMD_STOP);
        }
        return;
    }

    if (code == LV_EVENT_PRESSED) {
        g_stop_pressed = true;
        if (!g_stop_repeat_timer) {
            g_stop_repeat_timer = lv_timer_create(stop_repeat_cb, 250, nullptr);
        }
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        g_stop_pressed = false;
        if (g_stop_repeat_timer) {
            lv_timer_del(g_stop_repeat_timer);
            g_stop_repeat_timer = nullptr;
        }
        return;
    }
}

static void tare_small_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        send_cmd(UserCommand::CMD_TARE_SCALE);
    }
}

static void settings_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (scr_settings) lv_scr_load(scr_settings);
    }
}

static void power_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        send_cmd(UserCommand::CMD_SHELLY_TOGGLE);
    }
}

static void steam_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (g_state == STATE_STEAM_WARMUP || g_state == STATE_STEAM_READY) {
        send_cmd(UserCommand::CMD_EXIT_STEAM);
    } else {
        send_cmd(UserCommand::CMD_ENTER_STEAM);
    }
}

static void hot_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        send_cmd(UserCommand::CMD_FLUSH);
    }
}

static void clean_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        send_cmd(UserCommand::CMD_DESCALE);
    }
}

// Settings screen controls
static void back_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (scr_main) lv_scr_load(scr_main);
    }
}

static void temp_minus_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        g_brew_sp -= 0.5f;
        if (g_brew_sp < 85.0f) g_brew_sp = 85.0f;
        send_cmd(UserCommand::CMD_SET_BREW_TEMP, g_brew_sp);
    }
}

static void temp_plus_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        g_brew_sp += 0.5f;
        if (g_brew_sp > 105.0f) g_brew_sp = 105.0f;
        send_cmd(UserCommand::CMD_SET_BREW_TEMP, g_brew_sp);
    }
}

static void scale_minus_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        g_scale_fac -= 5.0f;
        send_cmd(UserCommand::CMD_SET_SCALE_CALIBRATION, g_scale_fac);
    }
}

static void scale_plus_event(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        g_scale_fac += 5.0f;
        send_cmd(UserCommand::CMD_SET_SCALE_CALIBRATION, g_scale_fac);
    }
}

// ----------------------------
// Build screens
// ----------------------------
static void build_main() {
    scr_main = lv_obj_create(NULL);
    style_root(scr_main);

    // Header area
    lbl_title = lv_label_create(scr_main);
    lv_label_set_text(lbl_title, "Gaggia E24");
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(0x111827), 0);
    lv_obj_set_pos(lbl_title, 12, 6);

    lbl_sub = lv_label_create(scr_main);
    lv_label_set_text(lbl_sub, "Ready");
    lv_obj_set_style_text_font(lbl_sub, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0x6B7280), 0);
    lv_obj_set_pos(lbl_sub, 12, 28);

    lbl_wifi = lv_label_create(scr_main);
    lv_label_set_text(lbl_wifi, "");
    lv_obj_set_style_text_font(lbl_wifi, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_wifi, lv_color_hex(0x10B981), 0);
    lv_obj_align(lbl_wifi, LV_ALIGN_TOP_MID, 0, 14);

    // Settings button (text + gear)
    btn_settings = lv_btn_create(scr_main);
    lv_obj_set_size(btn_settings, 92, 30);
    lv_obj_set_pos(btn_settings, 338, 8);
    lv_obj_set_style_radius(btn_settings, 12, 0);
    lv_obj_set_style_bg_color(btn_settings, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(btn_settings, 1, 0);
    lv_obj_set_style_border_color(btn_settings, lv_color_hex(0xE5E7EB), 0);
    lv_obj_add_event_cb(btn_settings, settings_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl_set = lv_label_create(btn_settings);
    lv_label_set_text(lbl_set, LV_SYMBOL_SETTINGS " Settings");
    lv_obj_set_style_text_font(lbl_set, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_set, lv_color_hex(0x111827), 0);
    lv_obj_center(lbl_set);

    // Power button
    btn_power = lv_btn_create(scr_main);
    lv_obj_set_size(btn_power, 34, 34);
    lv_obj_set_pos(btn_power, 436, 6);
    lv_obj_set_style_radius(btn_power, 17, 0);
    lv_obj_set_style_bg_color(btn_power, lv_color_hex(0xEF4444), 0);
    lv_obj_set_style_border_width(btn_power, 0, 0);
    lv_obj_add_event_cb(btn_power, power_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl_pwr = lv_label_create(btn_power);
    lv_label_set_text(lbl_pwr, LV_SYMBOL_POWER);
    lv_obj_set_style_text_font(lbl_pwr, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl_pwr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lbl_pwr);

    // Layout constants
    const int x0 = 10;
    const int y0 = 50;
    const int gap = 10;
    const int w_left = 160;
    const int w_mid  = 190;
    const int w_right = 90;
    const int h_card = 125;

    const int x_left = x0;
    const int x_mid  = x_left + w_left + gap;
    const int x_right = x_mid + w_mid + gap;

    // Temp card
    card_temp = make_card(scr_main, x_left, y0, w_left, h_card);

    lv_obj_t* lbl_temp_title = lv_label_create(card_temp);
    lv_label_set_text(lbl_temp_title, "Temp");
    lv_obj_set_style_text_font(lbl_temp_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_temp_title, lv_color_hex(0x111827), 0);
    lv_obj_align(lbl_temp_title, LV_ALIGN_TOP_LEFT, 0, 0);

    // Ready pill
    pill_temp = lv_obj_create(card_temp);
    lv_obj_set_size(pill_temp, 64, 20);
    lv_obj_set_style_radius(pill_temp, 10, 0);
    lv_obj_set_style_bg_color(pill_temp, lv_color_hex(0xDCFCE7), 0);
    lv_obj_set_style_border_width(pill_temp, 0, 0);
    lv_obj_align(pill_temp, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_clear_flag(pill_temp, LV_OBJ_FLAG_SCROLLABLE);

    lbl_pill_temp = lv_label_create(pill_temp);
    lv_label_set_text(lbl_pill_temp, "Ready");
    lv_obj_set_style_text_font(lbl_pill_temp, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_pill_temp, lv_color_hex(0x16A34A), 0);
    lv_obj_center(lbl_pill_temp);

    lbl_temp_value = lv_label_create(card_temp);
    lv_label_set_text(lbl_temp_value, "--C");
    lv_obj_set_style_text_font(lbl_temp_value, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_temp_value, lv_color_hex(0x16A34A), 0);
    lv_obj_align(lbl_temp_value, LV_ALIGN_CENTER, 0, -4);

    lbl_temp_target = lv_label_create(card_temp);
    lv_label_set_text(lbl_temp_target, "Target: --C");
    lv_obj_set_style_text_font(lbl_temp_target, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_temp_target, lv_color_hex(0x6B7280), 0);
    lv_obj_align(lbl_temp_target, LV_ALIGN_BOTTOM_MID, 0, 2);

    // Weight card (live scale reading)
    card_press = make_card(scr_main, x_left, y0 + h_card + gap, w_left, h_card);

    lv_obj_t* lbl_p_title = lv_label_create(card_press);
    lv_label_set_text(lbl_p_title, "Weight");
    lv_obj_set_style_text_font(lbl_p_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_p_title, lv_color_hex(0x111827), 0);
    lv_obj_align(lbl_p_title, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_press_value = lv_label_create(card_press);
    lv_label_set_text(lbl_press_value, "--.-");
    lv_obj_set_style_text_font(lbl_press_value, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_press_value, lv_color_hex(0x16A34A), 0);
    lv_obj_align(lbl_press_value, LV_ALIGN_CENTER, 0, -6);

    bar_press = lv_bar_create(card_press);
    lv_obj_set_size(bar_press, w_left - 24, 10);
    lv_obj_align(bar_press, LV_ALIGN_BOTTOM_MID, 0, -6);
    // 0..600g covers cup + shot use-case. Bar value is grams.
    lv_bar_set_range(bar_press, 0, 600);
    lv_bar_set_value(bar_press, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_press, lv_color_hex(0xE5E7EB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_press, lv_color_hex(0x22C55E), LV_PART_INDICATOR);

    // Small TARE button inside weight card
    btn_tare_small = lv_btn_create(card_press);
    lv_obj_set_size(btn_tare_small, 56, 26);
    lv_obj_align(btn_tare_small, LV_ALIGN_BOTTOM_RIGHT, 0, -22);
    lv_obj_set_style_radius(btn_tare_small, 10, 0);
    lv_obj_set_style_bg_color(btn_tare_small, lv_color_hex(0xF9FAFB), 0);
    lv_obj_set_style_border_width(btn_tare_small, 1, 0);
    lv_obj_set_style_border_color(btn_tare_small, lv_color_hex(0xE5E7EB), 0);
    lv_obj_add_event_cb(btn_tare_small, tare_small_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl_tare = lv_label_create(btn_tare_small);
    lv_label_set_text(lbl_tare, "Tare");
    lv_obj_set_style_text_font(lbl_tare, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_tare, lv_color_hex(0x111827), 0);
    lv_obj_center(lbl_tare);

    // Shot time card (blue)
    card_time = make_card(scr_main, x_mid, y0, w_mid, h_card);
    lv_obj_set_style_bg_color(card_time, lv_color_hex(0x2563EB), 0);
    lv_obj_set_style_border_width(card_time, 0, 0);

    lv_obj_t* lbl_time_title = lv_label_create(card_time);
    lv_label_set_text(lbl_time_title, LV_SYMBOL_REFRESH " Shot Time");
    lv_obj_set_style_text_font(lbl_time_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_time_title, lv_color_hex(0xE5E7EB), 0);
    lv_obj_align(lbl_time_title, LV_ALIGN_TOP_MID, 0, 2);

    lbl_time_value = lv_label_create(card_time);
    lv_label_set_text(lbl_time_value, "0.0s");
    lv_obj_set_style_text_font(lbl_time_value, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_time_value, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(lbl_time_value, LV_ALIGN_CENTER, 0, -2);

    lbl_weight_line = lv_label_create(card_time);
    lv_label_set_text(lbl_weight_line, "Weight --.- g");
    lv_obj_set_style_text_font(lbl_weight_line, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_weight_line, lv_color_hex(0xDBEAFE), 0);
    lv_obj_align(lbl_weight_line, LV_ALIGN_BOTTOM_MID, 0, 0);

    // STOP/BREW button tile
    btn_brewstop = lv_btn_create(scr_main);
    lv_obj_set_pos(btn_brewstop, x_mid, y0 + h_card + gap);
    lv_obj_set_size(btn_brewstop, w_mid, h_card);
    lv_obj_set_style_radius(btn_brewstop, 14, 0);
    lv_obj_set_style_bg_color(btn_brewstop, lv_color_hex(0xDC2626), 0);
    lv_obj_set_style_bg_opa(btn_brewstop, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn_brewstop, 0, 0);
    lv_obj_set_style_shadow_width(btn_brewstop, 18, 0);
    lv_obj_set_style_shadow_opa(btn_brewstop, LV_OPA_20, 0);
    lv_obj_set_style_shadow_ofs_y(btn_brewstop, 6, 0);
    lv_obj_add_event_cb(btn_brewstop, brewstop_event, LV_EVENT_ALL, nullptr);

    lbl_brewstop = lv_label_create(btn_brewstop);
    lv_label_set_text(lbl_brewstop, "STOP");
    lv_obj_set_style_text_font(lbl_brewstop, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lbl_brewstop, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lbl_brewstop);

    // Right column buttons
    btn_steam = make_small_btn(scr_main, w_right, 80, "Steam");
    lv_obj_set_pos(btn_steam, x_right, y0);
    lv_obj_add_event_cb(btn_steam, steam_event, LV_EVENT_CLICKED, nullptr);

    btn_hot = make_small_btn(scr_main, w_right, 80, "Hot\nWater");
    lv_obj_set_pos(btn_hot, x_right, y0 + 80 + gap);
    lv_obj_add_event_cb(btn_hot, hot_event, LV_EVENT_CLICKED, nullptr);

    btn_clean = make_small_btn(scr_main, w_right, 80, "Clean");
    lv_obj_set_pos(btn_clean, x_right, y0 + 2*(80 + gap));
    lv_obj_add_event_cb(btn_clean, clean_event, LV_EVENT_CLICKED, nullptr);
}

static void build_settings() {
    scr_settings = lv_obj_create(NULL);
    style_root(scr_settings);

    lv_obj_t* title = lv_label_create(scr_settings);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x111827), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Card 1: Brew temp
    lv_obj_t* c1 = make_card(scr_settings, 10, 50, 460, 110);
    lv_obj_t* t1 = lv_label_create(c1);
    lv_label_set_text(t1, "Brew Temp");
    lv_obj_set_style_text_font(t1, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(t1, lv_color_hex(0x111827), 0);
    lv_obj_align(t1, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_set_temp = lv_label_create(c1);
    lv_label_set_text(lbl_set_temp, "--.- C");
    lv_obj_set_style_text_font(lbl_set_temp, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lbl_set_temp, lv_color_hex(0x16A34A), 0);
    lv_obj_align(lbl_set_temp, LV_ALIGN_CENTER, 0, 10);

    lv_obj_t* bminus = lv_btn_create(c1);
    lv_obj_set_size(bminus, 70, 60);
    lv_obj_align(bminus, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_radius(bminus, 14, 0);
    lv_obj_set_style_bg_color(bminus, lv_color_hex(0x111827), 0);
    lv_obj_add_event_cb(bminus, temp_minus_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lm = lv_label_create(bminus);
    lv_label_set_text(lm, "-");
    lv_obj_set_style_text_font(lm, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lm, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lm);

    lv_obj_t* bplus = lv_btn_create(c1);
    lv_obj_set_size(bplus, 70, 60);
    lv_obj_align(bplus, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_radius(bplus, 14, 0);
    lv_obj_set_style_bg_color(bplus, lv_color_hex(0x111827), 0);
    lv_obj_add_event_cb(bplus, temp_plus_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lp = lv_label_create(bplus);
    lv_label_set_text(lp, "+");
    lv_obj_set_style_text_font(lp, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lp, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lp);

    // Card 2: Scale calibration factor
    lv_obj_t* c2 = make_card(scr_settings, 10, 170, 460, 110);
    lv_obj_t* t2 = lv_label_create(c2);
    lv_label_set_text(t2, "Scale Factor");
    lv_obj_set_style_text_font(t2, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(t2, lv_color_hex(0x111827), 0);
    lv_obj_align(t2, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_set_scale = lv_label_create(c2);
    lv_label_set_text(lbl_set_scale, "0.0");
    lv_obj_set_style_text_font(lbl_set_scale, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_set_scale, lv_color_hex(0x2563EB), 0);
    lv_obj_align(lbl_set_scale, LV_ALIGN_CENTER, 0, 10);

    lv_obj_t* sminus = lv_btn_create(c2);
    lv_obj_set_size(sminus, 70, 60);
    lv_obj_align(sminus, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_radius(sminus, 14, 0);
    lv_obj_set_style_bg_color(sminus, lv_color_hex(0x111827), 0);
    lv_obj_add_event_cb(sminus, scale_minus_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lsm = lv_label_create(sminus);
    lv_label_set_text(lsm, "-5");
    lv_obj_set_style_text_font(lsm, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lsm, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lsm);

    lv_obj_t* splus = lv_btn_create(c2);
    lv_obj_set_size(splus, 70, 60);
    lv_obj_align(splus, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_radius(splus, 14, 0);
    lv_obj_set_style_bg_color(splus, lv_color_hex(0x111827), 0);
    lv_obj_add_event_cb(splus, scale_plus_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lsp = lv_label_create(splus);
    lv_label_set_text(lsp, "+5");
    lv_obj_set_style_text_font(lsp, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lsp, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lsp);

    // Back
    lv_obj_t* back = lv_btn_create(scr_settings);
    lv_obj_set_size(back, 460, 34);
    lv_obj_align(back, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_radius(back, 14, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x111827), 0);
    lv_obj_add_event_cb(back, back_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lb = lv_label_create(back);
    lv_label_set_text(lb, "BACK");
    lv_obj_set_style_text_font(lb, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lb, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lb);
}

// ----------------------------
// Public API
// ----------------------------
void UI_Init(void) {
    build_main();
    build_settings();
    lv_scr_load(scr_main);
}

void UI_Update(const StateSnapshot& snap) {
    g_state = snap.state;
    g_fault = snap.fault;
    g_brew_sp = snap.brew_setpoint_c;
    g_scale_fac = snap.scale_factor;
    g_wifi_ok = snap.wifi_connected;

    // Header subtext
    if (snap.state == STATE_FAULT) {
        char fbuf[32];
        snprintf(fbuf, sizeof(fbuf), "Fault (%d)", (int)snap.fault);
        lv_label_set_text(lbl_sub, fbuf);
        lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0xDC2626), 0);
    } else {
        lv_label_set_text(lbl_sub, state_text(snap.state));
        lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0x6B7280), 0);
    }

    lv_label_set_text(lbl_wifi, g_wifi_ok ? "WiFi" : "No WiFi");
    lv_obj_set_style_text_color(lbl_wifi, g_wifi_ok ? lv_color_hex(0x10B981) : lv_color_hex(0xEF4444), 0);

    // Temp
    if (isfinite(snap.temp_c)) {
        char tb[16];
        snprintf(tb, sizeof(tb), "%.0f\xC2\xB0" "C", snap.temp_c);
        lv_label_set_text(lbl_temp_value, tb);
    } else {
        lv_label_set_text(lbl_temp_value, "--C");
    }

    char tgt[24];
    snprintf(tgt, sizeof(tgt), "Target: %.0f\xC2\xB0" "C", snap.brew_setpoint_c);
    lv_label_set_text(lbl_temp_target, tgt);

    bool temp_ready = isfinite(snap.temp_c) && fabsf(snap.temp_c - snap.brew_setpoint_c) < 1.5f;
    if (temp_ready && snap.state == STATE_IDLE && snap.fault == FAULT_NONE) {
        lv_label_set_text(lbl_pill_temp, "Ready");
        lv_obj_set_style_bg_color(pill_temp, lv_color_hex(0xDCFCE7), 0);
        lv_obj_set_style_text_color(lbl_pill_temp, lv_color_hex(0x16A34A), 0);
    } else {
        lv_label_set_text(lbl_pill_temp, "Heat");
        lv_obj_set_style_bg_color(pill_temp, lv_color_hex(0xE5E7EB), 0);
        lv_obj_set_style_text_color(lbl_pill_temp, lv_color_hex(0x6B7280), 0);
    }

    // Pressure
    // Live weight (shows even when not brewing)
    if (isfinite(snap.weight_g) && snap.weight_g > -999.0f) {
        char wb[16];
        snprintf(wb, sizeof(wb), "%.1f", snap.weight_g);
        lv_label_set_text(lbl_press_value, wb);
        int v = (int)lroundf(fminf(fmaxf(snap.weight_g, 0.0f), 600.0f));
        lv_bar_set_value(bar_press, v, LV_ANIM_OFF);
    } else {
        lv_label_set_text(lbl_press_value, "--.-");
        lv_bar_set_value(bar_press, 0, LV_ANIM_OFF);
    }

    // Shot time
    const float sec = (float)snap.shot_ms / 1000.0f;
    char sbuf[16];
    snprintf(sbuf, sizeof(sbuf), "%.1fs", sec);
    lv_label_set_text(lbl_time_value, sbuf);

    // Yield line (extracted) + target
    if (isfinite(snap.yield_g)) {
        char wbuf[64];
        if (isfinite(snap.target_yield_g) && snap.target_yield_g > 0.1f) {
            snprintf(wbuf, sizeof(wbuf), "Yield %.1f / %.0f g", snap.yield_g, snap.target_yield_g);
        } else {
            snprintf(wbuf, sizeof(wbuf), "Yield %.1f g", snap.yield_g);
        }
        lv_label_set_text(lbl_weight_line, wbuf);
    } else {
        lv_label_set_text(lbl_weight_line, "Yield --.- g");
    }

    // Brew/Stop tile visuals
    if (snap.state == STATE_IDLE && snap.fault == FAULT_NONE) {
        lv_label_set_text(lbl_brewstop, "BREW");
        lv_obj_set_style_bg_color(btn_brewstop, lv_color_hex(0x16A34A), 0);
    } else if (snap.state == STATE_FAULT) {
        lv_label_set_text(lbl_brewstop, "RESET\n(HOLD)");
        lv_obj_set_style_bg_color(btn_brewstop, lv_color_hex(0xDC2626), 0);
    } else {
        lv_label_set_text(lbl_brewstop, "STOP");
        lv_obj_set_style_bg_color(btn_brewstop, lv_color_hex(0xDC2626), 0);
    }

    // Enable/disable secondary actions
    const bool idle_ok = (snap.state == STATE_IDLE && snap.fault == FAULT_NONE);
    set_btn_enabled(btn_steam, idle_ok || snap.state == STATE_STEAM_WARMUP || snap.state == STATE_STEAM_READY);
    set_btn_enabled(btn_hot, idle_ok);
    set_btn_enabled(btn_clean, idle_ok);

    // Settings screen labels
    if (lbl_set_temp) {
        char spt[24];
        snprintf(spt, sizeof(spt), "%.1f C", g_brew_sp);
        lv_label_set_text(lbl_set_temp, spt);
    }
    if (lbl_set_scale) {
        char sc[24];
        snprintf(sc, sizeof(sc), "%.1f", g_scale_fac);
        lv_label_set_text(lbl_set_scale, sc);
    }
}

namespace UI {
    lv_obj_t* get_screen() { return scr_main; }
}
