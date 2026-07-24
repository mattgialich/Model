/* biped.c — two-legged robot, 3 actuated joints per leg (hip pitch, knee,
 * ankle pitch), 6 actuators total.
 *
 * Actuator tree:
 *   pelvis (root, torso geometry above)
 *     ├─ l_hip (revolute Y) ── l_knee (revolute Y) ── l_ankle (revolute Y, foot)
 *     └─ r_hip ── r_knee ── r_ankle
 *
 * Base propagation: kinematic twist integration, same as the quadruped.
 */
#include "robots.h"
#include <stdio.h>
#include <string.h>

/* --- design parameters ------------------------------------------------- */
static const double TORSO_W   = 0.24, TORSO_D = 0.12, TORSO_H = 0.40;
static const double HIP_Y     = 0.09;   /* hip lateral offset from pelvis */
static const double THIGH_LEN = 0.30;
static const double SHIN_LEN  = 0.30;
static const double FOOT_LEN  = 0.18, FOOT_W = 0.08, FOOT_H = 0.03;
static const double STAND_Z   = 0.62;   /* pelvis height when standing */

typedef struct { const char *prefix; double sy; double phase; } LegDef;
static const LegDef LEGS[2] = {
    {"l",  1, 0.0},
    {"r", -1, M_PI},
};

void biped_build(Robot *r) {
    robot_init(r);
    strncpy(r->name, "biped", ROBOT_NAME_LEN - 1);

    Link pelvis = link_make("pelvis", -1);
    pelvis.geom        = GEOM_BOX;
    pelvis.dims        = v3(TORSO_D, TORSO_W, TORSO_H);
    pelvis.geom_offset = v3(0, 0, TORSO_H / 2 + 0.02);  /* torso above hips */
    pelvis.mass        = 12.0;
    int b = robot_add_link(r, pelvis);

    for (int i = 0; i < 2; i++) {
        const LegDef *leg = &LEGS[i];
        char name[ROBOT_NAME_LEN];

        snprintf(name, sizeof name, "%s_hip", leg->prefix);
        Link hip = link_make(name, b);
        hip.joint      = JOINT_REVOLUTE;
        hip.origin_pos = v3(0, leg->sy * HIP_Y, 0);
        hip.axis       = v3(0, 1, 0);
        hip.act_mode   = ACT_POSITION;
        hip.limit_min  = -1.6; hip.limit_max = 1.6;
        hip.max_vel    = 10.0; hip.max_torque = 60.0;
        hip.geom       = GEOM_BOX;
        hip.dims       = v3(0.06, 0.06, THIGH_LEN);
        hip.geom_offset= v3(0, 0, -THIGH_LEN / 2);
        hip.mass       = 2.5;
        int h = robot_add_link(r, hip);

        snprintf(name, sizeof name, "%s_knee", leg->prefix);
        Link knee = link_make(name, h);
        knee.joint      = JOINT_REVOLUTE;
        knee.origin_pos = v3(0, 0, -THIGH_LEN);
        knee.axis       = v3(0, 1, 0);
        knee.act_mode   = ACT_POSITION;
        knee.limit_min  = 0.0; knee.limit_max = 2.4;   /* bends backward */
        knee.max_vel    = 10.0; knee.max_torque = 40.0;
        knee.geom       = GEOM_BOX;
        knee.dims       = v3(0.05, 0.05, SHIN_LEN);
        knee.geom_offset= v3(0, 0, -SHIN_LEN / 2);
        knee.mass       = 1.5;
        int k = robot_add_link(r, knee);

        snprintf(name, sizeof name, "%s_ankle", leg->prefix);
        Link ankle = link_make(name, k);
        ankle.joint      = JOINT_REVOLUTE;
        ankle.origin_pos = v3(0, 0, -SHIN_LEN);
        ankle.axis       = v3(0, 1, 0);
        ankle.act_mode   = ACT_POSITION;
        ankle.limit_min  = -0.8; ankle.limit_max = 0.8;
        ankle.max_vel    = 10.0; ankle.max_torque = 25.0;
        ankle.geom       = GEOM_BOX;
        ankle.dims       = v3(FOOT_LEN, FOOT_W, FOOT_H);
        ankle.geom_offset= v3(FOOT_LEN / 4, 0, -FOOT_H / 2);
        ankle.mass       = 0.6;
        robot_add_link(r, ankle);
    }

    r->base_pos    = v3(0, 0, STAND_Z);
    r->base_update = robot_base_update_twist;
}

/* --- demo controller: simple alternating walk cycle --------------------- */
static void biped_ctrl_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static void biped_ctrl_step(Controller *c, const Observation *obs,
                            double *cmd, int n_cmd, double base_cmd[3]) {
    (void)c; (void)n_cmd;
    double freq = 1.2;
    double phi  = 2 * M_PI * freq * obs->time;

    for (int i = 0; i < 2; i++) {
        double p     = phi + LEGS[i].phase;
        double swing = fmax(0.0, sin(p));
        double hip   = 0.20 + 0.30 * cos(p);
        double knee  = 0.25 + 0.65 * swing;
        double ankle = -(hip - knee) * 0.5;   /* keep the foot near level */
        cmd[3 * i + 0] = hip;
        cmd[3 * i + 1] = knee;
        cmd[3 * i + 2] = clampd(ankle, -0.8, 0.8);
    }

    base_cmd[0] = 0.35;    /* walk forward, m/s */
    base_cmd[1] = 0.0;
    base_cmd[2] = 0.0;
}

Controller *biped_demo_controller(void) {
    static Controller c = {
        .name  = "biped_walk",
        .reset = biped_ctrl_reset,
        .step  = biped_ctrl_step,
    };
    return &c;
}
