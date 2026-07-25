/* test_world.c — Unit tests for world.c functions */
#include "world.h"
#include "test_runner.h"
#include "robots.h"
#include <stdio.h>
#include <string.h>

/* Recording controller: captures the observation and writes known commands */
typedef struct {
    Observation last_obs;
    int steps;
} RecordCtx;

static void record_step(Controller *c, const Observation *obs,
                        double *cmd, int n_cmd, double *base_cmd) {
    (void)n_cmd; (void)base_cmd;
    RecordCtx *ctx = c->ctx;
    ctx->last_obs = *obs;
    for (int i = 0; i < n_cmd; i++)
        cmd[i] = 3.0;
    ctx->steps++;
}

static void record_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static Controller dummy_controller = {
    .name  = "record",
    .ctx   = NULL,
    .step  = record_step,
    .reset = record_reset
};

TEST(world_init) {
    World w;
    world_init(&w, 0.001);

    ASSERT_FLOAT_EQ(w.dt, 0.001, 1e-12);
    ASSERT_FLOAT_EQ(w.gravity, 9.81, 1e-9);
    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-9);
}

TEST(world_init_different_dt) {
    World w;
    world_init(&w, 0.01);

    ASSERT_FLOAT_EQ(w.dt, 0.01, 1e-12);
}

TEST(world_reset) {
    World w;
    world_init(&w, 0.01);

    w.time = 5.0;

    world_reset(&w);

    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-9);
}

TEST(world_step_no_controller) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    double initial_time = w.time;
    world_step(&w);

    ASSERT_FLOAT_EQ(w.time, initial_time + 0.01, 1e-12);
}

TEST(world_observation_contents) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    dummy_controller.ctx = &ctx;
    w.controller = &dummy_controller;

    world_step(&w);

    /* After first step, Observation.time should still be 0.0 (sim time before step) */
    ASSERT_FLOAT_EQ(ctx.last_obs.time, 0.0, 1e-12);
    ASSERT_EQ(ctx.steps, 1);
}

TEST(world_commands_reach_robot) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    dummy_controller.ctx = &ctx;
    w.controller = &dummy_controller;

    world_step(&w);

    /* Commands should have been set by record_step */
    for (int i = 0; i < w.robot.n_actuators; i++) {
        ASSERT_FLOAT_EQ(w.robot.cmd[i], 3.0, 1e-9);
    }
}

TEST(world_reset_clears_state) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    w.time = 5.0;

    world_reset(&w);

    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-9);
}

TEST(world_time_accumulates) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("car"));
    w.controller = NULL;

    for (int i = 0; i < 1000; i++)
        world_step(&w);

    ASSERT_FLOAT_EQ(w.time, 1.0, 1e-9);
}

TEST(world_time_accumulates_controller) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    dummy_controller.ctx = &ctx;
    w.controller = &dummy_controller;

    for (int i = 0; i < 100; i++)
        world_step(&w);

    ASSERT_FLOAT_EQ(w.time, 1.0, 1e-9);
    ASSERT_EQ(ctx.steps, 100);
}

/* Test function for main.c */
int world_tests(void) {
    printf("Testing world.c functions\n\n");

    RUN_TEST(world_init);
    RUN_TEST(world_init_different_dt);
    RUN_TEST(world_reset);
    RUN_TEST(world_step_no_controller);
    RUN_TEST(world_observation_contents);
    RUN_TEST(world_commands_reach_robot);
    RUN_TEST(world_reset_clears_state);
    RUN_TEST(world_time_accumulates);
    RUN_TEST(world_time_accumulates_controller);

    return 0;
}