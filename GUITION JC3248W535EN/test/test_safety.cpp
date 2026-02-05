#include <unity.h>
#include "logic/safety.h"

static void test_overtemp_fault() {
    SafetyConfig cfg; cfg.max_temp_c = 170.0f;
    SafetyInputs in; in.temp_c = 175.0f; in.zc_ok = true; in.weight_g = 0;
    TEST_ASSERT_EQUAL(FAULT_OVERTEMP, safety_check(in, cfg));
}

static void test_zc_missing_fault() {
    SafetyConfig cfg;
    SafetyInputs in; in.temp_c = 90.0f; in.zc_ok = false; in.weight_g = 0;
    TEST_ASSERT_EQUAL(FAULT_ZC_MISSING, safety_check(in, cfg));
}

static void test_tc_disconnect_fault() {
    SafetyConfig cfg;
    SafetyInputs in; in.temp_c = 999.0f; in.zc_ok = true; in.weight_g = 0;
    TEST_ASSERT_EQUAL(FAULT_TC_DISCONNECTED, safety_check(in, cfg));
}

void run_safety_tests() {
    RUN_TEST(test_overtemp_fault);
    RUN_TEST(test_zc_missing_fault);
    RUN_TEST(test_tc_disconnect_fault);
}
