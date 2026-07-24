/* test_registry.c — Unit tests for robots/registry.c */
#include "robots.h"
#include "test_runner.h"
#include <stdio.h>

TEST(registry_has_all_models) {
    ASSERT_EQ(N_ROBOT_MODELS, 3);
    for (int i = 0; i < N_ROBOT_MODELS; i++) {
        ASSERT_TRUE(ROBOT_MODELS[i].name != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].description != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].build != NULL);
        ASSERT_TRUE(ROBOT_MODELS[i].demo_controller != NULL);
    }
}

TEST(registry_find_by_name) {
    const RobotModel *car   = robot_model_find("car");
    const RobotModel *quad  = robot_model_find("quadruped");
    const RobotModel *biped = robot_model_find("biped");

    ASSERT_TRUE(car && quad && biped);
    ASSERT_STRING_EQ(car->name, "car");
    ASSERT_STRING_EQ(quad->name, "quadruped");
    ASSERT_STRING_EQ(biped->name, "biped");
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

    world_load_model(&w, robot_model_find("biped"));

    /* Rebuilding must fully replace the old robot and reset time */
    ASSERT_STRING_EQ(w.robot.name, "biped");
    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-12);
    ASSERT_EQ(w.robot.n_actuators, 6);
    ASSERT_FLOAT_EQ(w.robot.q[0], 0.0, 1e-12);
}

/* Test function for main.c */
int registry_tests(void) {
    printf("Testing robots/registry.c functions\n\n");

    RUN_TEST(registry_has_all_models);
    RUN_TEST(registry_find_by_name);
    RUN_TEST(registry_find_unknown);
    RUN_TEST(registry_load_model);
    RUN_TEST(registry_load_switches_models);

    return 0;
}
