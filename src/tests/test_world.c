/* test_world.c — Unit tests for world.c functions */
#include "world.h"
#include "test_runner.h"
#include "robots.h"
#include <stdio.h>
#include <string.h>

/* Dummy controller for testing */
static void dummy_step(Controller *c, const Observation *obs,
                       double *cmd, int n_cmd, double *base_cmd) {
    (void)c; (void)obs; (void)cmd; (void)n_cmd; (void)base_cmd;
}

static void dummy_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static Controller dummy_controller = {
    .step = dummy_step,
    .reset = dummy_reset
};

TEST(world_init) {
    World w;
    world_init(&w, 0.001);

    ASSERT_FLOAT_EQ(w.dt, 0.001, 1e-12);
    ASSERT_FLOAT_EQ(w.gravity, 9.81, 1e-9);
    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-9);
}

TEST(world_reset) {
    World w;
    world_init(&w, 0.01);

    /* Modify the world */
    w.time = 5.0;

    world_reset(&w);

    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-9);
}

TEST(world_step_no_controller) {
    World w;
    world_init(&w, 0.01);

    /* Load a simple robot */
    world_load_model(&w, &ROBOT_MODELS[0]); /* car */

    double initial_time = w.time;
    world_step(&w);

    ASSERT_FLOAT_EQ(w.time, initial_time + 0.01, 1e-12);
}

TEST(world_step_with_controller) {
    World w;
    world_init(&w, 0.01);

    /* Load a simple robot */
    world_load_model(&w, &ROBOT_MODELS[0]); /* car */

    w.controller = &dummy_controller;

    double initial_time = w.time;
    world_step(&w);

    ASSERT_FLOAT_EQ(w.time, initial_time + 0.01, 1e-12);
}

/* Recording controller: captures the observation and writes known commands */
typedef struct {
    Observation last_obs;
    int steps;
    int resets;
} RecordCtx;

static void record_step(Controller *c, const Observation *obs,
                        double *cmd, int n_cmd, double *base_cmd) {
    RecordCtx *ctx = c->ctx;
    ctx->last_obs = *obs;
    ctx->steps++;
    for (int i = 0; i < n_cmd; i++)
        cmd[i] = 3.0;              /* target velocity for every wheel */
    base_cmd[0] = 0.0;
}

static void record_reset(Controller *c, const Robot *r) {
    (void)r;
    ((RecordCtx *)c->ctx)->resets++;
}

TEST(world_observation_contents) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    Controller rec = {.name = "rec", .ctx = &ctx,
                      .reset = record_reset, .step = record_step};
    w.controller = &rec;

    world_step(&w);

    /* The controller saw the pre-step state of the world */
    ASSERT_EQ(ctx.steps, 1);
    ASSERT_FLOAT_EQ(ctx.last_obs.time, 0.0, 1e-12);
    ASSERT_EQ(ctx.last_obs.n_links, w.robot.n_links);
    ASSERT_EQ(ctx.last_obs.n_actuators, w.robot.n_actuators);
    ASSERT_TRUE(ctx.last_obs.q == w.robot.q);
    ASSERT_TRUE(ctx.last_obs.dq == w.robot.dq);
    ASSERT_TRUE(ctx.last_obs.act_link == w.robot.act_link);
}

TEST(world_commands_reach_robot) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    Controller rec = {.name = "rec", .ctx = &ctx,
                      .reset = record_reset, .step = record_step};
    w.controller = &rec;

    world_step(&w);

    /* Every velocity-mode actuator (the 4 wheels) should be spinning at
     * the commanded 3 rad/s after one step */
    Robot *r = &w.robot;
    int checked = 0;
    for (int a = 0; a < r->n_actuators; a++) {
        int i = r->act_link[a];
        if (r->links[i].act_mode == ACT_VELOCITY) {
            ASSERT_FLOAT_EQ(r->dq[i], 3.0, 1e-9);
            checked++;
        }
    }
    ASSERT_EQ(checked, 4);
}

TEST(world_reset_clears_state) {
    World w;
    world_init(&w, 0.01);
    world_load_model(&w, robot_model_find("car"));

    RecordCtx ctx = {0};
    Controller rec = {.name = "rec", .ctx = &ctx,
                      .reset = record_reset, .step = record_step};
    w.controller = &rec;

    for (int i = 0; i < 10; i++)
        world_step(&w);
    w.robot.base_cmd[0] = 9.9;

    world_reset(&w);

    ASSERT_FLOAT_EQ(w.time, 0.0, 1e-12);
    ASSERT_FLOAT_EQ(w.robot.cmd[0], 0.0, 1e-12);
    ASSERT_FLOAT_EQ(w.robot.base_cmd[0], 0.0, 1e-12);
    ASSERT_EQ(ctx.resets, 1);
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

/* Test function for main.c */
int world_tests(void) {
    printf("Testing world.c functions\n\n");

    RUN_TEST(world_init);
    RUN_TEST(world_reset);
    RUN_TEST(world_step_no_controller);
    RUN_TEST(world_step_with_controller);
    RUN_TEST(world_observation_contents);
    RUN_TEST(world_commands_reach_robot);
    RUN_TEST(world_reset_clears_state);
    RUN_TEST(world_time_accumulates);

    return 0;
}