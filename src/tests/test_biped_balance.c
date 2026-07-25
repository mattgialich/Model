/* test_biped_balance.c — Unit tests for biped balance control */
#include "robots.h"
#include "test_runner.h"
#include "../../robots/biped_balance.h"
#include <stdio.h>

/* Helper to create a simple robot for testing */
static void make_test_robot(Robot *r) {
    Link pelvis = link_make("pelvis", -1);
    robot_init(r);
    robot_add_link(r, pelvis);
}

TEST(biped_balance_compute_com) {
    Robot r;
    make_test_robot(&r);
    
    /* With a simple robot, COM should be at base_pos */
    Vec3 com = biped_compute_com(&r);
    
    /* Base position defaults to zero */
    ASSERT_FLOAT_EQ(com.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(com.y, 0.0, 1e-9);
}

TEST(biped_balance_state_initializes) {
    /* Verify balance state is defined */
    extern BalanceState biped_balance_state;
    
    ASSERT_FLOAT_EQ(biped_balance_state.last_com_x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(biped_balance_state.last_com_y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(biped_balance_state.zmp_error_sum, 0.0, 1e-9);
}

TEST(biped_balance_zmp_computes) {
    Robot r;
    make_test_robot(&r);
    
    /* ZMP should compute without crashing */
    Observation obs = {
        .time = 0.0,
        .n_links = r.n_links,
        .n_actuators = r.n_actuators,
        .q = r.q,
        .dq = r.dq,
        .act_link = r.act_link,
        .base_pos = r.base_pos,
        .base_rot = r.base_rot
    };
    
    Vec3 zmp = biped_compute_zmp(&r, &obs);
    
    ASSERT_FLOAT_EQ(zmp.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(zmp.y, 0.0, 1e-9);
}

TEST(biped_balance_apply_correction) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("biped"));
    
    /* Run a few steps to populate state */
    for (int i = 0; i < 10; i++) {
        world_step(&w);
    }

    /* Try balance correction - should not crash */
    Observation obs = {
        .time = w.time,
        .n_links = w.robot.n_links,
        .n_actuators = w.robot.n_actuators,
        .q = w.robot.q,
        .dq = w.robot.dq,
        .act_link = w.robot.act_link,
        .base_pos = w.robot.base_pos,
        .base_rot = w.robot.base_rot
    };
    biped_apply_balance_correction(w.robot.cmd, w.robot.n_actuators, &obs);

    ASSERT_TRUE(1);  /* Just verify it runs */
}

/* Test function for main.c */
int biped_balance_tests(void) {
    printf("Testing robots/biped_balance.c functions\n\n");

    RUN_TEST(biped_balance_compute_com);
    RUN_TEST(biped_balance_state_initializes);
    RUN_TEST(biped_balance_zmp_computes);
    RUN_TEST(biped_balance_apply_correction);

    return 0;
}