/* test_registry.c — Unit tests for robots/registry.c */
#include "robots.h"
#include "test_runner.h"
#include <stdio.h>

TEST(registry_has_all_models) {
    ASSERT_EQ(N_ROBOT_MODELS, 4);
    for (int i = 0; i < N_ROBOT_MODELS; i++) {
        ASSERT_TRUE(ROBOT_MODELS[i].name != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].description != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].build != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].demo_controller != NULL);
    }
}

TEST(registry_find_by_name) {
    const RobotModel *car      = robot_model_find("car");
    const RobotModel *quad     = robot_model_find("quadruped");
    const RobotModel *biped    = robot_model_find("biped");
    const RobotModel *humanoid = robot_model_find("humanoid");

    ASSERT_TRUE(car && quad && biped && humanoid);
    ASSERT_STRING_EQ(car->name, "car");
    ASSERT_STRING_EQ(quad->name, "quadruped");
    ASSERT_STRING_EQ(biped->name, "biped");
    ASSERT_STRING_EQ(humanoid->name, "humanoid");
}

TEST(registry_find_unknown) {
    ASSERT_TRUE(robot_model_find("hovercraft") == NULL);
    ASSERT_TRUE(robot_model_find("") == NULL);
}

TEST(registry_load_model) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("car"));

    ASSERT_STRING_EQ(w.robot.name, "car");
    ASSERT_TRUE(w.robot.n_links > 0);
    ASSERT_TRUE(w.controller != NULL);
    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-12);
}

TEST(registry_load_switches_models) {
    World w;
    world_init(&w, 0.001);

    world_load_model(&w, robot_model_find("car"));
    for (int i = 0; i < 100; i++)
        world_step(&w);

    world_load_model(&w, robot_model_find("humanoid"));

    /* Rebuilding must fully replace the old robot and reset time */
    ASSERT_STRING_EQ(w.robot.name, "humanoid");
    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-12);
    /* Humanoid has more actuators than the old biped */
    ASSERT_EQ(w.robot.n_actuators, 24);
    int l_hip = robot_find_link(&w.robot, "l_hip");
    ASSERT_FLOAT_EQ(w.robot.q[l_hip], 0.08, 1e-6);   /* Initial hip pitch */
}

TEST(registry_humanoid_has_arms) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("humanoid"));

    /* Humanoid should have arms */
    ASSERT_TRUE(w.robot.n_links > 20);   /* More than biped's links */
}

/* Test function for main.c */
int registry_tests(void) {
    printf("Testing robots/registry.c functions\n\n");

    RUN_TEST(registry_has_all_models);
    RUN_TEST(registry_find_by_name);
    RUN_TEST(registry_find_unknown);
    RUN_TEST(registry_load_model);
    RUN_TEST(registry_load_switches_models);
    RUN_TEST(registry_humanoid_has_arms);

    return 0;
}