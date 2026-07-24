/* test_robot.c — Unit tests for robot.c functions */
#include "robot.h"
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

    r.link_pos[0] = v3(1.0, 2.0, 3.0);
    r.link_rot[0] = quat_identity();

    robot_fk(&r);

    /* robot_fk calculates: link_pos = base_pos + rotate(base_rot, origin_pos)
       With default base_pos=(0,0,0), result should be origin_pos */
    ASSERT_FLOAT_EQ(r.link_pos[0].x, 0.5, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].z, 0.0, 1e-9);
}

TEST(robot_step_velocity_clamped) {
    Robot r;
    robot_init(&r);

    Link L = link_make("wheel", -1);
    L.joint = JOINT_WHEEL;
    L.act_mode = ACT_VELOCITY;
    L.max_vel = 2.0;
    robot_add_link(&r, L);

    /* Command far beyond the actuator's speed limit */
    r.cmd[0] = 100.0;
    robot_step(&r, 0.01);

    ASSERT_FLOAT_EQ(r.dq[0], 2.0, 1e-9);
    ASSERT_FLOAT_EQ(r.q[0], 0.02, 1e-9);
}

TEST(robot_step_position_chase_rate) {
    Robot r;
    robot_init(&r);

    Link L = link_make("joint", -1);
    L.joint = JOINT_REVOLUTE;
    L.act_mode = ACT_POSITION;
    L.limit_min = -M_PI;
    L.limit_max = M_PI;
    L.max_vel = 1.0;               /* slow actuator */
    robot_add_link(&r, L);

    /* Target is 1 rad away but the joint can only move max_vel * dt */
    r.cmd[0] = 1.0;
    robot_step(&r, 0.01);

    ASSERT_FLOAT_EQ(r.q[0], 0.01, 1e-9);
    ASSERT_FLOAT_EQ(r.dq[0], 1.0, 1e-9);
}

TEST(robot_step_wheel_wraps) {
    Robot r;
    robot_init(&r);

    Link L = link_make("wheel", -1);
    L.joint = JOINT_WHEEL;
    L.act_mode = ACT_VELOCITY;
    L.max_vel = 40.0;
    robot_add_link(&r, L);

    /* Spin fast for many steps; angle must stay numerically bounded */
    r.cmd[0] = 40.0;
    for (int i = 0; i < 10000; i++)
        robot_step(&r, 0.001);

    ASSERT_TRUE(fabs(r.q[0]) <= 2 * M_PI + 1e-9);
    ASSERT_FLOAT_EQ(r.dq[0], 40.0, 1e-9);
}

TEST(robot_step_velocity_stops_at_limit) {
    Robot r;
    robot_init(&r);

    Link L = link_make("joint", -1);
    L.joint = JOINT_REVOLUTE;
    L.act_mode = ACT_VELOCITY;
    L.max_vel = 10.0;
    L.limit_min = -0.05;
    L.limit_max = 0.05;
    robot_add_link(&r, L);

    /* Drive into the hard stop: q clamps to the limit and dq zeroes */
    r.cmd[0] = 10.0;
    robot_step(&r, 0.01);

    ASSERT_FLOAT_EQ(r.q[0], 0.05, 1e-9);
    ASSERT_FLOAT_EQ(r.dq[0], 0.0, 1e-9);
}

TEST(robot_fk_chain_rotation) {
    Robot r;
    robot_init(&r);

    /* Revolute joint at the base, child link 1m out along parent X */
    Link L0 = link_make("shoulder", -1);
    L0.joint = JOINT_REVOLUTE;
    L0.axis = v3(0, 0, 1);
    robot_add_link(&r, L0);

    Link L1 = link_make("arm", 0);
    L1.origin_pos = v3(1.0, 0.0, 0.0);
    robot_add_link(&r, L1);

    /* Rotate the shoulder 90 degrees: the arm should swing from +X to +Y */
    r.q[0] = M_PI / 2.0;
    robot_fk(&r);

    ASSERT_FLOAT_EQ(r.link_pos[1].x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[1].y, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[1].z, 0.0, 1e-9);
}

TEST(robot_fk_prismatic) {
    Robot r;
    robot_init(&r);

    Link L = link_make("slider", -1);
    L.joint = JOINT_PRISMATIC;
    L.axis = v3(0, 0, 1);
    robot_add_link(&r, L);

    r.q[0] = 0.5;
    robot_fk(&r);

    ASSERT_FLOAT_EQ(r.link_pos[0].x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].z, 0.5, 1e-9);
}

TEST(robot_fk_base_offset) {
    Robot r;
    robot_init(&r);

    Link L = link_make("link", -1);
    L.origin_pos = v3(0.5, 0.0, 0.0);
    robot_add_link(&r, L);

    r.base_pos = v3(1.0, 2.0, 3.0);
    robot_fk(&r);

    ASSERT_FLOAT_EQ(r.link_pos[0].x, 1.5, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].y, 2.0, 1e-9);
    ASSERT_FLOAT_EQ(r.link_pos[0].z, 3.0, 1e-9);
}

TEST(robot_base_twist_forward) {
    Robot r;
    robot_init(&r);
    r.base_pos = v3(0, 0, 0.26);

    /* 1 m/s forward for 0.1 s: +0.1 m in X, height unchanged */
    r.base_cmd[0] = 1.0;
    robot_base_update_twist(&r, 0.1);

    ASSERT_FLOAT_EQ(r.base_pos.x, 0.1, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.z, 0.26, 1e-9);
}

TEST(robot_base_twist_yaw) {
    Robot r;
    robot_init(&r);

    /* pi/2 rad/s yaw for 1 s: base turns 90 degrees in place */
    r.base_cmd[2] = M_PI / 2.0;
    robot_base_update_twist(&r, 1.0);

    ASSERT_FLOAT_EQ(quat_yaw(r.base_rot), M_PI / 2.0, 1e-9);
    ASSERT_FLOAT_EQ(v3_len(r.base_pos), 0.0, 1e-9);
}

TEST(robot_base_twist_lateral_after_yaw) {
    Robot r;
    robot_init(&r);

    /* Face +Y, then command body-frame forward: motion is world +Y */
    r.base_cmd[2] = M_PI / 2.0;
    robot_base_update_twist(&r, 1.0);
    r.base_cmd[2] = 0.0;
    r.base_cmd[0] = 1.0;
    robot_base_update_twist(&r, 0.5);

    ASSERT_FLOAT_EQ(r.base_pos.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.y, 0.5, 1e-9);
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
    RUN_TEST(robot_step_velocity_clamped);
    RUN_TEST(robot_step_position_chase_rate);
    RUN_TEST(robot_step_wheel_wraps);
    RUN_TEST(robot_step_velocity_stops_at_limit);

    RUN_TEST(robot_fk_simple);
    RUN_TEST(robot_fk_chain_rotation);
    RUN_TEST(robot_fk_prismatic);
    RUN_TEST(robot_fk_base_offset);

    RUN_TEST(robot_base_twist_forward);
    RUN_TEST(robot_base_twist_yaw);
    RUN_TEST(robot_base_twist_lateral_after_yaw);

    return 0;
}
