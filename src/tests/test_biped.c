/* test_biped.c — Unit tests for robots/biped.c */
#include "robots.h"
#include "test_runner.h"
#include <stdio.h>

static const double BIPED_STAND_Z = 0.62;   /* mirrors STAND_Z in biped.c */

TEST(biped_build_structure) {
    Robot r;
    robot_model_find("biped")->build(&r);

    ASSERT_STRING_EQ(r.name, "biped");
    ASSERT_EQ(r.n_links, 7);        /* pelvis + 2 legs x (hip, knee, ankle) */
    ASSERT_EQ(r.n_actuators, 6);
    ASSERT_FLOAT_EQ(r.base_pos.z, BIPED_STAND_Z, 1e-12);
    ASSERT_TRUE(r.base_update != NULL);

    const char *sides[] = {"l", "r"};
    char name[32];
    for (int i = 0; i < 2; i++) {
        snprintf(name, sizeof name, "%s_hip", sides[i]);
        int hip = robot_find_link(&r, name);
        snprintf(name, sizeof name, "%s_knee", sides[i]);
        int knee = robot_find_link(&r, name);
        snprintf(name, sizeof name, "%s_ankle", sides[i]);
        int ankle = robot_find_link(&r, name);

        ASSERT_TRUE(hip >= 0 && knee >= 0 && ankle >= 0);
        ASSERT_EQ(r.links[knee].parent, hip);
        ASSERT_EQ(r.links[ankle].parent, knee);
    }
}

TEST(biped_knee_bends_one_way) {
    Robot r;
    robot_model_find("biped")->build(&r);

    /* Human-style knee: position limits allow only backward flexion */
    int knee = robot_find_link(&r, "l_knee");
    ASSERT_FLOAT_EQ(r.links[knee].limit_min, 0.0, 1e-12);
    ASSERT_TRUE(r.links[knee].limit_max > 0);
}

TEST(biped_demo_respects_limits) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("biped"));

    for (int i = 0; i < 2000; i++) {           /* 2 s of walking */
        world_step(&w);
        for (int a = 0; a < w.robot.n_actuators; a++) {
            int li = w.robot.act_link[a];
            const Link *L = &w.robot.links[li];
            if (w.robot.q[li] < L->limit_min - 1e-9 ||
                w.robot.q[li] > L->limit_max + 1e-9) {
                ASSERT_TRUE(0);                /* joint out of limits */
                return;
            }
        }
    }
    ASSERT_TRUE(1);
}

TEST(biped_demo_walks_forward) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("biped"));

    for (int i = 0; i < 2000; i++)
        world_step(&w);

    Robot *r = &w.robot;
    ASSERT_TRUE(isfinite(r->base_pos.x) && isfinite(r->base_pos.y));
    ASSERT_TRUE(r->base_pos.x > 0.3);          /* ~0.35 m/s commanded */
    ASSERT_FLOAT_EQ(r->base_pos.y, 0.0, 1e-9); /* no lateral drift */
    ASSERT_FLOAT_EQ(r->base_pos.z, BIPED_STAND_Z, 1e-12);
}

/* Test function for main.c */
int biped_tests(void) {
    printf("Testing robots/biped.c functions\n\n");

    RUN_TEST(biped_build_structure);
    RUN_TEST(biped_knee_bends_one_way);
    RUN_TEST(biped_demo_respects_limits);
    RUN_TEST(biped_demo_walks_forward);

    return 0;
}
