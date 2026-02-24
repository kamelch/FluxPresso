#include <unity.h>
#include "logic/espresso_sm.h"

static void test_brew_sequence_transitions() {
    EspressoSM sm;
    sm.reset();

    ShotProfile p;
    p.preinfuse_ms = 1000;
    p.preinfuse_pump = 0.5f;
    p.bloom_ms = 500;
    p.bloom_pump = 0.0f;
    p.brew_ms = 1000;
    p.brew_pump = 0.7f;

    sm.start_brew(0);
    TEST_ASSERT_EQUAL(STATE_BREW_PREHEAT, sm.state());

    // Warm enough -> PREINFUSE
    sm.step(100, 93.0f, p, 93.0f);
    TEST_ASSERT_EQUAL(STATE_BREW_PREINFUSE, sm.state());

    // After preinfuse -> HOLD
    sm.step(1200, 93.0f, p, 93.0f);
    TEST_ASSERT_EQUAL(STATE_BREW_HOLD, sm.state());

    // After bloom -> FLOW
    sm.step(1800, 93.0f, p, 93.0f);
    TEST_ASSERT_EQUAL(STATE_BREW_FLOW, sm.state());

    // After brew -> DRAIN
    sm.step(3000, 93.0f, p, 93.0f);
    TEST_ASSERT_EQUAL(STATE_BREW_DRAIN, sm.state());

    // After drain -> IDLE
    sm.step(3900, 93.0f, p, 93.0f);
    TEST_ASSERT_EQUAL(STATE_IDLE, sm.state());
}

void run_state_machine_tests() {
    RUN_TEST(test_brew_sequence_transitions);
}
