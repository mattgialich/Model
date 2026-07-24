/* quadruped.c — four-legged robot, 2 actuated joints per leg (hip pitch +
 * knee), 8 actuators total.
 *
 * Actuator tree:
 *   body (root)
 *     ├─ fl_hip (revolute Y, position) ── fl_knee (revolute Y, position)
 *     ├─ fr_hip ── fr_knee
 *     ├─ rl_hip ── rl_knee
 *     └─ rr_hip ── rr_knee
 *
 * Base propagation: kinematic — the gait controller commands a body twist
 * (base_cmd) which is integrated at constant stand height. Full dynamics
 * with ground contact replaces this later.
 */
#include "robots.h"
#include <stdio.h>
#include <string.h>

/* --- design parameters ------------------------------------------------- */
static const double BODY_LEN  = 0.45, BODY_WID = 0.22, BODY_HGT = 0.10;
static const double HIP_X     = 0.18;   /* hip fore/aft offset from center */
static const double HIP_Y     = 0.13;   /* hip lateral offset */
static const double UPPER_LEN = 0.16;   /* thigh length, m */
static const double LOWER_LEN = 0.16;   /* shin length, m */
static const double STAND_Z   = 0.26;   /* base height when standing */

/* stance pose */
static const double HIP0  =  0.35;
static const double KNEE0 = -0.80;

typedef struct { const char *prefix; double sx, sy; double phase; } LegDef;
/* diagonal pairs share phase: trot */
static const LegDef LEGS[4] = {
    {"fl",  1,  1, 0.0},
    {"fr",  1, -1, M_PI},
    {"rl", -1,  1, M_PI},
    {"rr", -1, -1, 0.0},
};

void quadruped_build(Robot *r) {
    robot_init(r);
    strncpy(r->name, "quadruped", ROBOT_NAME_LEN - 1);

    Link body = link_make("body", -1);
    body.geom = GEOM_BOX;
    body.dims = v3(BODY_LEN, BODY_WID, BODY_HGT);
    body.mass = 6.0;
    int b = robot_add_link(r, body);

    for (int i = 0; i < 4; i++) {
        const LegDef *leg = &LEGS[i];
        char name[ROBOT_NAME_LEN];

        snprintf(name, sizeof name, "%s_hip", leg->prefix);
        Link hip = link_make(name, b);
        hip.joint      = JOINT_REVOLUTE;
        hip.origin_pos = v3(leg->sx * HIP_X, leg->sy * HIP_Y, 0);
        hip.axis       = v3(0, 1, 0);
        hip.act_mode   = ACT_POSITION;
        hip.limit_min  = -1.4; hip.limit_max = 1.4;
        hip.max_vel    = 12.0; hip.max_torque = 20.0;
        hip.geom       = GEOM_BOX;
        hip.dims       = v3(0.045, 0.045, UPPER_LEN);
        hip.geom_offset= v3(0, 0, -UPPER_LEN / 2);
        hip.mass       = 0.8;
        int h = robot_add_link(r, hip);

        snprintf(name, sizeof name, "%s_knee", leg->prefix);
        Link knee = link_make(name, h);
        knee.joint      = JOINT_REVOLUTE;
        knee.origin_pos = v3(0, 0, -UPPER_LEN);
        knee.axis       = v3(0, 1, 0);
        knee.act_mode   = ACT_POSITION;
        knee.limit_min  = -2.4; knee.limit_max = 0.0;
        knee.max_vel    = 12.0; knee.max_torque = 15.0;
        knee.geom       = GEOM_BOX;
        knee.dims       = v3(0.035, 0.035, LOWER_LEN);
        knee.geom_offset= v3(0, 0, -LOWER_LEN / 2);
        knee.mass       = 0.4;
        robot_add_link(r, knee);
    }

    r->base_pos    = v3(0, 0, STAND_Z);
    r->base_update = robot_base_update_twist;
}

/* --- demo controller: trot gait ---------------------------------------- */
static void quad_ctrl_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static void quad_ctrl_step(Controller *c, const Observation *obs,
                           double *cmd, int n_cmd, double base_cmd[3]) {
    (void)c; (void)n_cmd;
    double freq = 1.8;                       /* strides per second */
    double phi  = 2 * M_PI * freq * obs->time;

    for (int i = 0; i < 4; i++) {
        double p     = phi + LEGS[i].phase;
        double swing = fmax(0.0, sin(p));    /* >0 during swing phase */
        cmd[2 * i + 0] = HIP0 + 0.28 * cos(p);      /* hip sweep */
        cmd[2 * i + 1] = KNEE0 - 0.55 * swing;      /* knee flex in swing */
    }

    base_cmd[0] = 0.45;                      /* walk forward, m/s */
    base_cmd[1] = 0.0;
    base_cmd[2] = 0.25 * sin(0.3 * obs->time);  /* lazy wander */
}

Controller *quadruped_demo_controller(void) {
    static Controller c = {
        .name  = "quadruped_trot",
        .reset = quad_ctrl_reset,
        .step  = quad_ctrl_step,
    };
    return &c;
}
