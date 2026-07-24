/* test_car.c — Unit tests for robots/car.c (build + bicycle-model base) */
#include "robots.h"
#include "test_runner.h"
#include <stdio.h>

static const double CAR_WHEEL_R = 0.06;   /* mirrors WHEEL_R in car.c */

/* actuator index driving the named link, -1 if not actuated */
static int car_act_index(const Robot *r, const char *link_name) {
    int li = robot_find_link(r, link_name);
    for (int a = 0; a < r->n_actuators; a++)
        if (r->act_link[a] == li) return a;
    return -1;
}

static void car_build_fresh(Robot *r) {
    robot_model_find("car")->build(r);
}

TEST(car_build_structure) {
    Robot r;
    car_build_fresh(&r);

    ASSERT_STRING_EQ(r.name, "car");
    ASSERT_EQ(r.n_links, 7);        /* chassis + 2 knuckles + 4 wheels */
    ASSERT_EQ(r.n_actuators, 6);    /* 2 steer + 4 wheel */
    ASSERT_FLOAT_EQ(r.base_pos.z, CAR_WHEEL_R, 1e-12);
    ASSERT_TRUE(r.base_update != NULL);

    const char *links[] = {"chassis", "steer_fl", "steer_fr",
                           "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    for (int i = 0; i < 7; i++)
        ASSERT_TRUE(robot_find_link(&r, links[i]) >= 0);
}

TEST(car_build_joint_config) {
    Robot r;
    car_build_fresh(&r);

    const Link *steer = &r.links[robot_find_link(&r, "steer_fl")];
    ASSERT_EQ(steer->joint, JOINT_REVOLUTE);
    ASSERT_EQ(steer->act_mode, ACT_POSITION);
    ASSERT_TRUE(steer->limit_max > 0 && steer->limit_min == -steer->limit_max);

    const Link *wheel = &r.links[robot_find_link(&r, "wheel_rl")];
    ASSERT_EQ(wheel->joint, JOINT_WHEEL);
    ASSERT_EQ(wheel->act_mode, ACT_VELOCITY);
    ASSERT_TRUE(wheel->max_vel > 0);
}

TEST(car_drives_straight) {
    Robot r;
    car_build_fresh(&r);

    /* All wheels at 10 rad/s, steering centered: 0.6 m/s straight ahead */
    const char *wheels[] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    for (int i = 0; i < 4; i++)
        r.cmd[car_act_index(&r, wheels[i])] = 10.0;

    for (int i = 0; i < 1000; i++)
        robot_step(&r, 0.001);

    ASSERT_FLOAT_EQ(r.base_pos.x, 10.0 * CAR_WHEEL_R * 1.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(quat_yaw(r.base_rot), 0.0, 1e-9);
    ASSERT_FLOAT_EQ(r.base_pos.z, CAR_WHEEL_R, 1e-12);
}

TEST(car_turns_with_steering) {
    Robot r;
    car_build_fresh(&r);

    const char *wheels[] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    for (int i = 0; i < 4; i++)
        r.cmd[car_act_index(&r, wheels[i])] = 10.0;
    r.cmd[car_act_index(&r, "steer_fl")] = 0.3;
    r.cmd[car_act_index(&r, "steer_fr")] = 0.3;

    for (int i = 0; i < 1000; i++)
        robot_step(&r, 0.001);

    /* Left steer at forward speed: yaw increases and the car curves left */
    ASSERT_TRUE(quat_yaw(r.base_rot) > 0.1);
    ASSERT_TRUE(r.base_pos.y > 0.01);
    ASSERT_TRUE(r.base_pos.x > 0.1);
}

TEST(car_wheel_speed_clamped) {
    Robot r;
    car_build_fresh(&r);

    int a = car_act_index(&r, "wheel_rl");
    int li = r.act_link[a];
    r.cmd[a] = 1000.0;
    robot_step(&r, 0.001);

    ASSERT_FLOAT_EQ(r.dq[li], r.links[li].max_vel, 1e-9);
}

TEST(car_demo_controller_moves) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("car"));

    for (int i = 0; i < 1000; i++)
        world_step(&w);

    Robot *r = &w.robot;
    ASSERT_TRUE(isfinite(r->base_pos.x) && isfinite(r->base_pos.y));
    ASSERT_TRUE(r->base_pos.x > 0.5);            /* drove forward ~1.2 m/s */
    ASSERT_FLOAT_EQ(r->base_pos.z, CAR_WHEEL_R, 1e-12);
}

/* Test function for main.c */
int car_tests(void) {
    printf("Testing robots/car.c functions\n\n");

    RUN_TEST(car_build_structure);
    RUN_TEST(car_build_joint_config);
    RUN_TEST(car_drives_straight);
    RUN_TEST(car_turns_with_steering);
    RUN_TEST(car_wheel_speed_clamped);
    RUN_TEST(car_demo_controller_moves);

    return 0;
}
