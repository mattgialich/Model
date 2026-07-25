/* test_robot.c — Unit tests for robot.c functions */
#include "../include/robot.h"
#include "test_runner.h"
#include <stdio.h>
#include <string.h>

TEST(robot_link_make) {
    Link L = link_make("test_link", -1);

    ASSERT_EQ(L.parent, -1);
    ASSERT_STRING_EQ(L.name, "test_link");
    ASSERT_EQ(L.joint, JOINT_FIXED);
    ASSERT_FLOAT_EQ(quat_yaw(L.origin_rot), 0.0, 1e-9);
    ASSERT_FLOAT_EQ(v3_len(L.axis), 1.0, 1e-9);
    ASSERT_FLOAT_EQ(L.origin_pos.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(L.origin_pos.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(L.origin_pos.z, 0.0, 1e-9);
}

TEST(robot_init) {
    Robot r;
    robot_init(&r);

    ASSERT_FLOAT_EQ(r.base_pos.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.z, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(quat_yaw(r.base_rot), 0.0, 1e-9);
    ASSERT_EQ(r.n_links, 0);
    ASSERT_EQ(r.n_actuators, 0);
}

TEST(robot_find_link) {
    Robot r;
    robot_init(&r);

    Link L1 = link_make("link1", -1);
    robot_add_link(&r, L1);

    Link L2 = link_make("link2", 0);
    robot_add_link(&r, L2);

    int idx1 = robot_find_link(&r, "link1");
    int idx2 = robot_find_link(&r, "link2");
    int idx3 = robot_find_link(&r, "nonexistent");

    ASSERT_EQ(idx1, 0);
    ASSERT_EQ(idx2, 1);
    ASSERT_EQ(idx3, -1);
}

TEST(robot_add_actuator) {
    Robot r;
    robot_init(&r);

    Link L = link_make("actuator", -1);
    L.act_mode = ACT_VELOCITY;

    int idx = robot_add_link(&r, L);

    ASSERT_EQ(r.n_actuators, 1);
    ASSERT_EQ(r.act_link[0], idx);
}

TEST(robot_step_velocity) {
    Robot r;
    robot_init(&r);

    Link L = link_make("wheel", -1);
    L.joint = JOINT_WHEEL;
    L.act_mode = ACT_VELOCITY;
    L.max_vel = 10.0;

    robot_add_link(&r, L);

    r.cmd[0] = 5.0;
    robot_step(&r, 0.01);

    ASSERT_FLOAT_EQ(r.q[0], 0.05, 1e-9);
    ASSERT_FLOAT_EQ(r.dq[0], 5.0, 1e-9);
}

TEST(robot_step_position) {
    Robot r;
    robot_init(&r);

    Link L = link_make("joint", -1);
    L.joint = JOINT_REVOLUTE;
    L.act_mode = ACT_POSITION;
    L.limit_min = -M_PI;
    L.limit_max = M_PI;

    robot_add_link(&r, L);

    r.cmd[0] = M_PI / 2.0;
    robot_step(&r, 0.01);

    /* Velocity = (target - current) / dt = (π/2 - 0) / 0.01 = 50π */
    /* After one step: q = 0 + v * dt = 50π * 0.01 = 0.5π */
    ASSERT_FLOAT_EQ(r.q[0], M_PI / 2.0, 1e-9);
}

TEST(robot_step_limit) {
    Robot r;
    robot_init(&r);

    Link L = link_make("joint", -1);
    L.joint = JOINT_REVOLUTE;
    L.act_mode = ACT_POSITION;
    L.limit_min = -1.0;
    L.limit_max = 1.0;

    robot_add_link(&r, L);

    /* Attempt to set beyond limit */
    r.cmd[0] = 5.0;
    robot_step(&r, 0.01);

    /* Should be clamped to limit */
    ASSERT_FLOAT_EQ(r.q[0], 1.0, 1e-9);
}

TEST(robot_fk_simple) {
    Robot r;
    robot_init(&r);

    Link L = link_make("link1", -1);
    L.origin_pos = v3(0.5, 0.0, 0.0);
    L.origin_rot = quat_identity();

    robot_add_link(&r, L);

    robot_fk(&r);

    /* robot_fk calculates: link_pos = base_pos + rotate(base_rot, origin_pos) */
    /* With default base_pos=(0,0,0), result should be origin_pos */
    ASSERT_FLOAT_EQ(r.link_pos[0].x, 0.5, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].z, 0.0, 1e-9);
}

/* Test function for main.c */
int robot_tests(void) {
    printf("Testing robot.c functions\n\n");

    RUN_TEST(robot_link_make);
    RUN_TEST(robot_init);
    RUN_TEST(robot_find_link);
    RUN_TEST(robot_add_actuator);

    RUN_TEST(robot_step_velocity);
    RUN_TEST(robot_step_position);
    RUN_TEST(robot_step_limit);

    RUN_TEST(robot_fk_simple);

    return 0;
}
