#include <unity.h>

// Forward declarations from other test translation units
void run_pid_tests();
void run_state_machine_tests();
void run_safety_tests();
void run_temp_predictor_tests();
void run_dreamsteam_tests();
void run_shot_history_tests();
void run_grind_advisor_tests();
void run_utils_tests();

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    run_pid_tests();
    run_state_machine_tests();
    run_safety_tests();
    run_temp_predictor_tests();
    run_dreamsteam_tests();
    run_shot_history_tests();
    run_grind_advisor_tests();
    run_utils_tests();
    return UNITY_END();
}
