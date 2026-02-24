#include <unity.h>
#include "utils/flow_shaper.h"
#include "utils/pump_ramp.h"
#include "utils/sensor_filters.h"

static void test_flow_shaper_ramp_up_and_down() {
    FlowShaperConfig c;
    c.enable = true;
    c.ramp_up_ms = 1000;
    c.ramp_down_ms = 1000;
    c.end_fraction = 0.5f;

    // At start, should be low but not zero (>=8% of base)
    uint8_t v0 = flow_shaper_apply(80, 0, 5000, c);
    TEST_ASSERT_TRUE(v0 >= 6); // 80 * 0.08 = 6.4

    // Mid ramp up ~50%
    uint8_t v1 = flow_shaper_apply(80, 500, 5000, c);
    TEST_ASSERT_TRUE(v1 > v0);

    // In steady state (elapsed > ramp_up, remaining > ramp_down)
    uint8_t v2 = flow_shaper_apply(80, 2000, 5000, c);
    TEST_ASSERT_EQUAL_UINT8(80, v2);

    // Near end ramp down (remaining small) -> lower than base
    uint8_t v3 = flow_shaper_apply(80, 4000, 500, c);
    TEST_ASSERT_TRUE(v3 < 80);
    TEST_ASSERT_TRUE(v3 >= 40); // end_fraction 0.5
}

static void test_pump_ramp_moves_toward_target() {
    PumpRampController pr;
    // First call initializes timestamp and returns current (0)
    TEST_ASSERT_EQUAL_UINT8(0, pr.setTarget(100, 1000));
    // Advance time => should increase
    uint8_t p = pr.setTarget(100, 1100);
    TEST_ASSERT_TRUE(p > 0);
    // Eventually reaches target
    for (int i = 0; i < 20 && !pr.isAtTarget(); ++i) {
        p = pr.setTarget(100, 1100 + (i+2)*100);
    }
    TEST_ASSERT_TRUE(pr.isAtTarget());
    TEST_ASSERT_EQUAL_UINT8(100, pr.getCurrentPower());
}

static void test_moving_average_filter() {
    MovingAverageFilter<int, 3> f;
    TEST_ASSERT_EQUAL_INT(1, f.update(1));
    TEST_ASSERT_EQUAL_INT(1, f.update(1));
    TEST_ASSERT_EQUAL_INT(2, f.update(4)); // (1+1+4)/3 = 2
    TEST_ASSERT_TRUE(f.isReady());
    TEST_ASSERT_EQUAL_INT(3, f.update(4)); // (1+4+4)/3 = 3
}

static void test_median_filter_rejects_spike() {
    MedianFilter3<int> f;
    f.update(10);
    f.update(10);
    f.update(1000); // spike
    // Need another sample to get median result (count>=3 returns median)
    int m = f.update(10);
    TEST_ASSERT_EQUAL_INT(10, m);
}

static void test_exponential_filter_smoothing() {
    ExponentialFilter<float> f(0.2f);
    float v1 = f.update(0.0f);
    float v2 = f.update(10.0f);
    TEST_ASSERT_TRUE(v2 > v1);
    TEST_ASSERT_TRUE(v2 < 10.0f);
}

void run_utils_tests() {
    RUN_TEST(test_flow_shaper_ramp_up_and_down);
    RUN_TEST(test_pump_ramp_moves_toward_target);
    RUN_TEST(test_moving_average_filter);
    RUN_TEST(test_median_filter_rejects_spike);
    RUN_TEST(test_exponential_filter_smoothing);
}
