/* test_quadruped.c — Unit tests for robots/quadruped.c */
#include "robots.h"
#include "test_runner.h"
#include <stdio.h>

static const double QUAD_STAND_Z = 0.26;   /* mirrors STAND_Z in quadruped.c */

TEST(quadruped_build_structure) {
    Robot r;
    robot_model_find("quadruped")->build(&r);

    ASSERT_STRING_EQ(r.name, "quadruped");
    ASSERT_EQ(r.n_links, 9);        /* body + 4 legs x (hip + knee) */
    ASSERT_EQ(r.n_actuators, 8);
    ASSERT_FLOAT_EQ(r.base_pos.z, QUAD_STAND_Z, 1e-12);
    ASSERT_TRUE(r.base_update != NULL);

    const char *legs[] = {"fl", "fr", "rl", "rr"};
    char name[32];
    for (int i = 0; i < 4; i++) {
        snprintf(name, sizeof name, "%s_hip", legs[i]);
        int hip = robot_find_link(&r, name);
        snprintf(name, sizeof name, "%s_knee", legs[i]);
        int knee = robot_find_link(&r, name);
        ASSERT_TRUE(hip >= 0 && knee >= 0);
        ASSERT_EQ(r.links[knee].parent, hip);
        ASSERT_EQ(r.links[hip].act_mode, ACT_POSITION);
        ASSERT_EQ(r.links[knee].act_mode, ACT_POSITION);
    }
}

TEST(quadruped_demo_respects_limits) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("quadruped"));

    for (int i = 0; i < 2000; i++) {           /* 2 s of trotting */
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

TEST(quadruped_demo_walks_forward) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("quadruped"));

    for (int i = 0; i < 2000; i++)
        world_step(&w);

    Robot *r = &w.robot;
    ASSERT_TRUE(isfinite(r->base_pos.x) && isfinite(r->base_pos.y));
    ASSERT_TRUE(r->base_pos.x > 0.4);          /* ~0.45 m/s commanded */
    ASSERT_FLOAT_EQ(r->base_pos.z, QUAD_STAND_Z, 1e-12);
}

TEST(quadruped_fk_feet_below_body) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("quadruped"));

    for (int i = 0; i < 500; i++)
        world_step(&w);

    /* Knee (shin) links must sit below the body while trotting */
    Robot *r = &w.robot;
    const char *legs[] = {"fl", "fr", "rl", "rr"};
    char name[32];
    for (int i = 0; i < 4; i++) {
        snprintf(name, sizeof name, "%s_knee", legs[i]);
        int knee = robot_find_link(r, name);
        ASSERT_TRUE(r->link_pos[knee].z < r->base_pos.z);
    }
}

/* Test function for main.c */
int quadruped_tests(void) {
    printf("Testing robots/quadruped.c functions\n\n");

    RUN_TEST(quadruped_build_structure);
    RUN_TEST(quadruped_demo_respects_limits);
    RUN_TEST(quadruped_demo_walks_forward);
    RUN_TEST(quadruped_fk_feet_below_body);

    return 0;
}
