#include <unity.h>
#include "controllers/dreamsteam_controller.h"

static DreamSteamController::Config make_conservative_cfg() {
    DreamSteamController::Config c;
    c.mode = DreamSteamController::Mode::MODE_CONSERVATIVE;
    c.pulse_power_min = 20;
    c.pulse_power_max = 40;
    c.pulse_duration_ms = 200;
    c.pulse_interval_ms = 500;
    c.max_cumulative_pump_ms_per_min = 10000;
    c.min_temp_for_refill_c = 130.0f;
    c.max_temp_for_refill_c = 150.0f;
    c.temp_load_threshold_c = 2.0f;
    c.temp_drop_rate_threshold = -0.8f;
    c.max_continuous_pulses = 8;
    c.use_steam_valve_switch = false; // native tests
    return c;
}

static void test_dreamsteam_starts_and_ends_pulse_on_load() {
    DreamSteamController ds(make_conservative_cfg());

    const float setpoint = 145.0f;

    // Prime history
    ds.update(144.5f, setpoint, 0, true);
    ds.update(144.0f, setpoint, 300, true);
    ds.update(142.0f, setpoint, 600, true); // now below setpoint-2 => load

    // At 600ms still within pulse interval since last_pulse=0 -> 600<500? Actually 600>=500, so eligible.
    uint8_t out = ds.update(142.0f, setpoint, 600, true);
    TEST_ASSERT_EQUAL_UINT8(20, out);

    // During pulse, should keep output
    out = ds.update(141.8f, setpoint, 700, true);
    TEST_ASSERT_EQUAL_UINT8(20, out);

    // After duration, should stop
    out = ds.update(141.7f, setpoint, 900, true);
    TEST_ASSERT_EQUAL_UINT8(0, out);
}

static void test_dreamsteam_respects_temp_window_and_zc() {
    auto cfg = make_conservative_cfg();
    DreamSteamController ds(cfg);
    const float setpoint = 145.0f;

    // Too cold -> no refill
    uint8_t out = ds.update(120.0f, setpoint, 1000, true);
    TEST_ASSERT_EQUAL_UINT8(0, out);

    // Good temp but zc missing -> no refill
    out = ds.update(140.0f, setpoint, 2000, false);
    TEST_ASSERT_EQUAL_UINT8(0, out);
}

static void test_dreamsteam_cumulative_limit_blocks() {
    auto cfg = make_conservative_cfg();
    cfg.max_cumulative_pump_ms_per_min = 150; // very small
    cfg.pulse_duration_ms = 100;
    cfg.pulse_interval_ms = 0;
    DreamSteamController ds(cfg);
    const float setpoint = 145.0f;

    // Force load each time
    uint8_t out = ds.update(140.0f, setpoint, 1000, true);
    TEST_ASSERT_TRUE(out > 0);
    ds.update(140.0f, setpoint, 1100, true); // end pulse
    ds.update(140.0f, setpoint, 1200, true); // start again
    ds.update(140.0f, setpoint, 1300, true); // end pulse

    // At this point cumulative should be >= 200ms -> blocked
    out = ds.update(140.0f, setpoint, 1400, true);
    TEST_ASSERT_EQUAL_UINT8(0, out);
}

void run_dreamsteam_tests() {
    RUN_TEST(test_dreamsteam_starts_and_ends_pulse_on_load);
    RUN_TEST(test_dreamsteam_respects_temp_window_and_zc);
    RUN_TEST(test_dreamsteam_cumulative_limit_blocks);
}
