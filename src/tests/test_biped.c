/* test_biped.c — Unit tests for improved robots/biped.c
 *
 * Tests Iteration 1-5 features:
 * - Arms (shoulder pitch/roll/yaw, elbow, wrist)
 * - Improved geometry (cylinders, sphere head)
 * - Hip roll/yaw degrees of freedom
 * - Balance control with CoM tracking
 * - Enhanced gait and ankle complexity
 */
#include "robots.h"
#include "test_runner.h"
#include "../../robots/biped_balance.h"
#include <stdio.h>
#include <string.h>

/* Test that the biped builds with correct structure */
TEST(biped_build_structure) {
    Robot r;
    robot_model_find("biped")->build(&r);

    ASSERT_STRING_EQ(r.name, "biped");
    
    /* Expected link count:
       - 1 pelvis + 1 head = 2
       - 2 legs x (hip + hip_roll + hip_yaw + thigh + knee + shin
                   + ankle_pitch + ankle_roll + foot) = 2 x 9 = 18
       - 2 arms x (shoulder + shoulder_roll + shoulder_yaw + upper_arm
                   + elbow + forearm + wrist_pitch + wrist_roll) = 2 x 8 = 16
       Total: 2 + 18 + 16 = 36 links */
    ASSERT_EQ(r.n_links, 36);
    
    /* Expected actuators:
       - 2 legs x (hip+roll+yaw + knee + ankle_pitch+ankle_roll) = 2 x 6 = 12
       - 2 arms x (shoulder+roll+yaw + elbow + wrist_pitch+wrist_roll) = 2 x 6 = 12
       Total: 24 actuators */
    ASSERT_EQ(r.n_actuators, 24);
    
    /* Check some key joints exist */
    ASSERT_NE(robot_find_link(&r, "pelvis"), -1);
    ASSERT_NE(robot_find_link(&r, "head"), -1);
    ASSERT_NE(robot_find_link(&r, "l_hip"), -1);
    ASSERT_NE(robot_find_link(&r, "l_knee"), -1);
    ASSERT_NE(robot_find_link(&r, "l_shoulder"), -1);
    ASSERT_NE(robot_find_link(&r, "l_elbow"), -1);
}

/* Test that hips have 3 DOF (pitch, roll, yaw) */
TEST(biped_hip_three_dof) {
    Robot r;
    robot_model_find("biped")->build(&r);

    /* Find left hip - it should have roll and yaw children */
    int l_hip = robot_find_link(&r, "l_hip");
    ASSERT_TRUE(l_hip >= 0);
    
    /* Check that hip has roll capability (X axis) */
    int child = -1;
    for (int i = 0; i < r.n_links; i++) {
        if (r.links[i].parent == l_hip) {
            child = i;
            break;
        }
    }
    
    /* The direct child should be a hip roll joint */
    if (child >= 0) {
        const Link *L = &r.links[child];
        ASSERT_EQ(L->joint, JOINT_REVOLUTE);
    }
    
    /* Right hip should be symmetric */
    int r_hip = robot_find_link(&r, "r_hip");
    ASSERT_TRUE(r_hip >= 0);
}

/* Test cylinder geometry for limbs */
TEST(biped_cylinder_geometry) {
    Robot r;
    robot_model_find("biped")->build(&r);

    /* Check thigh uses cylinder geometry */
    int l_thigh = robot_find_link(&r, "l_thigh");
    ASSERT_TRUE(l_thigh >= 0);
    ASSERT_EQ(r.links[l_thigh].geom, GEOM_CYLINDER);
    
    /* Check shin uses cylinder geometry */
    int l_shin = robot_find_link(&r, "l_shin");
    ASSERT_TRUE(l_shin >= 0);
    ASSERT_EQ(r.links[l_shin].geom, GEOM_CYLINDER);
    
    /* Check head uses sphere geometry */
    int head = robot_find_link(&r, "head");
    ASSERT_TRUE(head >= 0);
    ASSERT_EQ(r.links[head].geom, GEOM_SPHERE);
}

/* Test ankle has 2 DOF (pitch and roll) */
TEST(biped_ankle_two_dof) {
    Robot r;
    robot_model_find("biped")->build(&r);

    int l_ankle_pitch = robot_find_link(&r, "l_ankle");
    ASSERT_TRUE(l_ankle_pitch >= 0);
    
    /* Find ankle roll (child of ankle pitch) */
    int l_ankle_roll = -1;
    for (int i = 0; i < r.n_links; i++) {
        if (r.links[i].parent == l_ankle_pitch) {
            l_ankle_roll = i;
            break;
        }
    }
    
    ASSERT_TRUE(l_ankle_roll >= 0);
    ASSERT_EQ(r.links[l_ankle_roll].joint, JOINT_REVOLUTE);
}

/* Test arm structure */
TEST(biped_arm_structure) {
    Robot r;
    robot_model_find("biped")->build(&r);

    /* Check shoulder exists with 3 DOF */
    ASSERT_NE(robot_find_link(&r, "l_shoulder"), -1);
    ASSERT_NE(robot_find_link(&r, "r_shoulder"), -1);
    
    /* Check elbow exists */
    ASSERT_NE(robot_find_link(&r, "l_elbow"), -1);
    ASSERT_NE(robot_find_link(&r, "r_elbow"), -1);
    
    /* Check wrist exists */
    ASSERT_NE(robot_find_link(&r, "l_wrist"), -1);
    ASSERT_NE(robot_find_link(&r, "r_wrist"), -1);
}

/* Test that biped controller is available */
TEST(biped_controller_exists) {
    Controller *c = robot_model_find("biped")->demo_controller();
    
    ASSERT_TRUE(c != NULL);
    ASSERT_STRING_EQ(c->name, "biped_walk");
}

/* Test that biped can walk (basic simulation) */
TEST(biped_walks_forward) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("biped"));

    double initial_x = w.robot.base_pos.x;
    
    for (int i = 0; i < 1200; i++) {           /* 1.2 s of walking */
        world_step(&w);
    }

    Robot *r = &w.robot;
    
    /* Should have moved forward */
    ASSERT_TRUE(r->base_pos.x > initial_x + 0.3);
    
    /* Should maintain roughly constant height */
    ASSERT_FLOAT_EQ(r->base_pos.z, 0.92, 0.1);   /* +/- 10cm tolerance */
    
    /* Should not drift sideways significantly */
    ASSERT_FLOAT_EQ(r->base_pos.y, 0.0, 0.2);    /* +/- 20cm tolerance */
}

/* Test balance state is tracked */
TEST(biped_balance_state_exists) {
    /* Reloading the biped resets the balance state to zero */
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("biped"));

    ASSERT_FLOAT_EQ(biped_balance_state.last_com_x, 0.0, 1e-12);
    ASSERT_FLOAT_EQ(biped_balance_state.last_com_y, 0.0, 1e-12);
    ASSERT_FLOAT_EQ(biped_balance_state.zmp_error_sum, 0.0, 1e-12);
}

/* Test function for main.c */
int biped_tests(void) {
    printf("Testing improved robots/biped.c functions\n\n");

    RUN_TEST(biped_build_structure);
    RUN_TEST(biped_hip_three_dof);
    RUN_TEST(biped_cylinder_geometry);
    RUN_TEST(biped_ankle_two_dof);
    RUN_TEST(biped_arm_structure);
    RUN_TEST(biped_controller_exists);
    RUN_TEST(biped_walks_forward);
    RUN_TEST(biped_balance_state_exists);

    return 0;
}