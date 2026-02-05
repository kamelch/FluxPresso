#include <unity.h>
#include "controllers/pid_controller.h"

static void test_pid_step_response_increases_output() {
    PIDController::Config c;
    c.kp = 2.0f;
    c.ki = 0.0f;
    c.kd = 0.0f;
    c.output_min = 0.0f;
    c.output_max = 100.0f;

    PIDController pid(c);
    pid.setSetpoint(100.0f);

    float o1 = pid.update(0.0f, 0);
    float o2 = pid.update(0.0f, 100);

    TEST_ASSERT_TRUE(o1 >= 0.0f);
    TEST_ASSERT_TRUE(o2 >= o1);
}

static void test_pid_output_limits() {
    PIDController::Config c;
    c.kp = 1000.0f;
    c.output_min = 0.0f;
    c.output_max = 10.0f;

    PIDController pid(c);
    pid.setSetpoint(100.0f);
    // Many PID implementations intentionally return 0 on the very first call (dt=0 / not initialized).
    // Call once to initialize internal time, then assert clamping on a subsequent update.
    (void)pid.update(0.0f, 0);
    float o = pid.update(0.0f, 100);
    TEST_ASSERT_TRUE(o <= 10.0f);
    TEST_ASSERT_TRUE(o >= 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, o);
}

void test_pid_step_response_increases_output_wrapper() { test_pid_step_response_increases_output(); }
void test_pid_output_limits_wrapper() { test_pid_output_limits(); }

// PlatformIO Unity runner will auto-discover these tests if we register them.
void run_pid_tests() {
    RUN_TEST(test_pid_step_response_increases_output_wrapper);
    RUN_TEST(test_pid_output_limits_wrapper);
}
