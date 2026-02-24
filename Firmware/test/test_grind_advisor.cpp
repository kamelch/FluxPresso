#include <unity.h>
#include "utils/grind_advisor.h"

static void test_grind_advisor_learns_and_suggests_best() {
    GrindAdvisor ga;
    TEST_ASSERT_TRUE(ga.begin());

    // Profile 0, medium roast
    const uint8_t profile = 0;
    const RoastType roast = ROAST_MEDIUM;

    // Teach: grind 10 is good (near target), grind 12 is worse
    for (int i = 0; i < 8; ++i) {
        ga.observe(profile, roast, 10, 38.0f, 38.0f, 28.0f); // great
    }
    for (int i = 0; i < 8; ++i) {
        ga.observe(profile, roast, 12, 30.0f, 38.0f, 40.0f); // poor
    }

    uint16_t out_grind = 0;
    uint8_t conf = 0;
    int8_t delta = 0;
    ga.suggest(profile, roast, 12, out_grind, conf, delta);

    TEST_ASSERT_EQUAL_UINT16(10, out_grind);
    TEST_ASSERT_TRUE(conf > 0);
    TEST_ASSERT_TRUE(delta < 0); // 10 is finer than 12
}

static void test_grind_advisor_handles_empty_model() {
    GrindAdvisor ga;
    TEST_ASSERT_TRUE(ga.begin());

    uint16_t out_grind = 0;
    uint8_t conf = 0;
    int8_t delta = 0;
    ga.suggest(0, ROAST_LIGHT, 15, out_grind, conf, delta);

    TEST_ASSERT_EQUAL_UINT16(15, out_grind);
    TEST_ASSERT_EQUAL_UINT8(0, conf);
    TEST_ASSERT_EQUAL_INT8(0, delta);
}

void run_grind_advisor_tests() {
    RUN_TEST(test_grind_advisor_learns_and_suggests_best);
    RUN_TEST(test_grind_advisor_handles_empty_model);
}
