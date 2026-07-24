/* main.c — entry point for the unit test suite */
#define TEST_RUNNER_MAIN   /* defines the shared test-runner state + reporter */
#include "test_runner.h"

extern int math3d_tests(void);
extern int robot_tests(void);
extern int world_tests(void);
extern int registry_tests(void);
extern int car_tests(void);
extern int quadruped_tests(void);
extern int biped_tests(void);
extern int biped_balance_tests(void);

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    printf("========================================\n");
    printf("Unit Tests for Model\n");
    printf("========================================\n\n");

    init_timing();

    math3d_tests();
    robot_tests();
    world_tests();
    registry_tests();
    car_tests();
    quadruped_tests();
    biped_tests();
    biped_balance_tests();

    return run_all_tests();
}