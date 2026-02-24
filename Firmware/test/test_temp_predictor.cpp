#include <unity.h>
#include "utils/temp_predictor.h"

static void test_temp_predictor_ready_and_estimate() {
    TempPredictor tp;
    TEST_ASSERT_FALSE(tp.isReady());

    tp.update(90.0f, 0);
    TEST_ASSERT_TRUE(tp.isReady());

    // 100% heater, default max heating rate 2C/s => 10C diff => ~5s
    const uint32_t ms = tp.estimateTimeToTarget(100.0f, 100.0f);
    TEST_ASSERT_TRUE(ms >= 4000);
    TEST_ASSERT_TRUE(ms <= 6000);

    // already above target => 0
    tp.update(105.0f, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, tp.estimateTimeToTarget(100.0f, 100.0f));
}

static void test_temp_predictor_predict_trend() {
    TempPredictor tp;
    tp.update(90.0f, 0);

    const float p1 = tp.predict(2000, 0.0f);     // cooling
    const float p2 = tp.predict(2000, 100.0f);   // heating
    TEST_ASSERT_TRUE(p2 > p1);
    TEST_ASSERT_TRUE(p2 >= 90.0f);
}

void run_temp_predictor_tests() {
    RUN_TEST(test_temp_predictor_ready_and_estimate);
    RUN_TEST(test_temp_predictor_predict_trend);
}
