#include <unity.h>

// Forward declarations from other test translation units
void run_pid_tests();
void run_state_machine_tests();
void run_safety_tests();

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    run_pid_tests();
    run_state_machine_tests();
    run_safety_tests();
    return UNITY_END();
}
