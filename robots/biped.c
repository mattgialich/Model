/* biped.c — improved two-legged humanoid with arms, balance control.
 *
 * Iteration 1: Added arms (shoulder pitch/roll/yaw, elbow, wrist)
 * Iteration 2: Improved geometry (cylinders for limbs, sphere head)
 * Iteration 3: Added hip roll/yaw degrees of freedom
 * Iteration 4: Enhanced balance control with CoM tracking
 * Iteration 5: Improved gait and ankle complexity
 *
 * Actuator tree (24 actuators total, 6 per limb):
 *   pelvis (root, torso + head geometry)
 *     ├─ l_hip (3-DOF: pitch/roll/yaw) ── l_knee ── l_ankle (2-DOF: pitch/roll)
 *     ├─ r_hip (3-DOF) ── r_knee ── r_ankle (2-DOF)
 *     ├─ l_shoulder (3-DOF) ── l_elbow ── l_wrist (2-DOF: pitch/roll)
 *     └─ r_shoulder (3-DOF) ── r_elbow ── r_wrist (2-DOF)
 *
 * The actuator index layout is shared with the balance controller via the
 * BIP_LEG_BASE / BIP_ARM_BASE macros in biped_balance.h.
 *
 * Base propagation: kinematic twist integration with balance control
 */
#include "robots.h"
#include "biped_balance.h"
#include <stdio.h>
#include <string.h>

/* --- design parameters ------------------------------------------------- */
/* Body dimensions (meters) */
static const double TORSO_W   = 0.26;    /* torso width (shoulder width) */
static const double TORSO_D   = 0.18;    /* torso depth */
static const double TORSO_H   = 0.45;    /* torso height (pelvis to shoulders) */
static const double HEAD_DIA  = 0.18;    /* head diameter */

/* Hip and leg dimensions */
static const double HIP_Y     = 0.10;    /* hip lateral offset (wider than human) */
static const double HIP_X     = 0.03;    /* hip anterior/posterior offset */
static const double THIGH_LEN = 0.40;    /* thigh length */
static const double SHIN_LEN  = 0.42;    /* shin length (longer shins) */
static const double THIGH_DIA = 0.06;    /* thigh diameter (cylinder) */
static const double SHIN_DIA  = 0.05;    /* shin diameter (cylinder) */

/* Foot and ankle dimensions */
static const double FOOT_LEN  = 0.22;    /* foot length (larger) */
static const double FOOT_W    = 0.10;    /* foot width (wider base) */
static const double FOOT_H    = 0.03;    /* foot height */

/* Arm dimensions */
static const double SHOULDER_Y = 0.18;   /* shoulder height from pelvis */
static const double UPPER_ARM_LEN = 0.32;
static const double FOREARM_LEN   = 0.30;
static const double UPPER_ARM_DIA = 0.05;
static const double FOREARM_DIA   = 0.04;

/* Mass properties (kg) */
static const double PELVIS_MASS   = 15.0;
static const double THIGH_MASS    = 4.5;
static const double SHIN_MASS     = 3.5;
static const double FOOT_MASS     = 1.0;
static const double ARM_UPPER_MASS= 2.5;
static const double ARM_FOREARM_MASS = 1.8;
static const double HEAD_MASS     = 3.0;

/* Joint limits and capabilities */
static const double MAX_VEL_HIP    = 12.0;
static const double MAX_VEL_KNEE   = 15.0;
static const double MAX_VEL_ANKLE  = 12.0;
static const double MAX_VEL_SHOULDER= 14.0;
static const double MAX_VEL_ELBOW  = 18.0;

static const double MAX_TQ_HIP     = 70.0;
static const double MAX_TQ_KNEE    = 80.0;
static const double MAX_TQ_ANKLE   = 40.0;
static const double MAX_TQ_SHOULDER= 35.0;
static const double MAX_TQ_ELBOW   = 25.0;

/* Standing configuration */
static const double STAND_Z      = 0.92; /* pelvis height when standing */
static const double STAND_HIP_Y  = 0.05; /* slight hip offset for stability */
static const double PELVIS_PITCH_STABLE = 0.08; /* slight forward lean for stability */

typedef struct { const char *prefix; double sy; double phase; } LegDef;
typedef struct { const char *prefix; double sx; double sy; } ArmDef;

static const LegDef LEGS[2] = {
    {"l",  1.0, 0.0},   /* left leg: sy=+ (right side) */
    {"r", -1.0, M_PI},  /* right leg: sy=- (left side) */
};

static const ArmDef ARMS[2] = {
    {"l",  1.0, -1.0},  /* left arm: sx=+, sy=- (front) */
    {"r", -1.0, -1.0},  /* right arm: sx=-, sy=- (front) */
};

/* Helper to create cylinder geometry for human-like limbs */
static Link link_make_cylinder(const char *name, int parent,
                                double length, double diameter) {
    Link L = link_make(name, parent);
    L.geom       = GEOM_CYLINDER;
    L.dims       = v3(diameter, length, diameter);  /* cyl: radius, height, radius */
    L.geom_offset= v3(0, -length / 2, 0);  /* origin at top of cylinder */
    return L;
}

/* Helper to create sphere geometry for head */
static Link link_make_sphere(const char *name, int parent, double diameter) {
    Link L = link_make(name, parent);
    L.geom       = GEOM_SPHERE;
    L.dims       = v3(diameter / 2, 0, 0);  /* sphere radius in x */
    L.geom_offset= v3(0, 0, diameter / 2);
    return L;
}

void biped_build(Robot *r) {
    robot_init(r);
    strncpy(r->name, "humanoid", ROBOT_NAME_LEN - 1);

    /* --- PELVIS (root) --- */
    Link pelvis = link_make("pelvis", -1);
    pelvis.geom        = GEOM_BOX;
    pelvis.dims        = v3(TORSO_D, TORSO_W, TORSO_H);
    pelvis.geom_offset = v3(0, 0, -TORSO_H / 2);  /* origin at pelvis center */
    pelvis.mass        = PELVIS_MASS;
    int b = robot_add_link(r, pelvis);

    /* --- HEAD (on top of pelvis) --- */
    Link head = link_make_sphere("head", b, HEAD_DIA);
    head.geom_offset = v3(0, 0, TORSO_H / 2 + HEAD_DIA / 2);
    head.mass        = HEAD_MASS;
    robot_add_link(r, head);

    /* --- LEGS --- */
    for (int i = 0; i < 2; i++) {
        const LegDef *leg = &LEGS[i];
        char name[ROBOT_NAME_LEN];

        /* Hip - 3 DOF: Y (pitch), Z (yaw), X (roll) */
        snprintf(name, sizeof name, "%s_hip", leg->prefix);
        Link hip = link_make_cylinder(name, b, THIGH_LEN + 0.02, 0.12);
        hip.joint      = JOINT_REVOLUTE;
        hip.origin_pos = v3(HIP_X, leg->sy * HIP_Y, 0);
        hip.axis       = v3(0, 1, 0);  /* Y-axis for pitch */
        hip.act_mode   = ACT_POSITION;
        hip.limit_min  = -1.8; hip.limit_max = 1.8;     /* pitch range */
        hip.max_vel    = MAX_VEL_HIP;
        hip.max_torque = MAX_TQ_HIP;
        hip.mass       = THIGH_MASS / 2;  /* shared between left/right */
        int h = robot_add_link(r, hip);

        /* Add roll capability (X axis) */
        snprintf(name, sizeof name, "%s_hip_roll", leg->prefix);
        Link hip_roll = link_make(name, h);
        hip_roll.joint      = JOINT_REVOLUTE;
        hip_roll.origin_pos = v3(0, 0, 0);
        hip_roll.axis       = v3(1, 0, 0);  /* X-axis for roll */
        hip_roll.act_mode   = ACT_POSITION;
        hip_roll.limit_min  = -0.8; hip_roll.limit_max = 0.8;
        hip_roll.max_vel    = MAX_VEL_HIP * 0.8;
        hip_roll.max_torque = MAX_TQ_HIP * 0.7;
        hip_roll.mass       = THIGH_MASS / 4;
        int hr = robot_add_link(r, hip_roll);

        /* Add yaw capability (Z axis) */
        snprintf(name, sizeof name, "%s_hip_yaw", leg->prefix);
        Link hip_yaw = link_make(name, hr);
        hip_yaw.joint      = JOINT_REVOLUTE;
        hip_yaw.origin_pos = v3(0, 0, 0);
        hip_yaw.axis       = v3(0, 0, 1);  /* Z-axis for yaw */
        hip_yaw.act_mode   = ACT_POSITION;
        hip_yaw.limit_min  = -0.6; hip_yaw.limit_max = 0.6;
        hip_yaw.max_vel    = MAX_VEL_HIP * 0.6;
        hip_yaw.max_torque = MAX_TQ_HIP * 0.5;
        hip_yaw.mass       = THIGH_MASS / 4;
        int hy = robot_add_link(r, hip_yaw);

        /* Thigh (cylinder) */
        snprintf(name, sizeof name, "%s_thigh", leg->prefix);
        Link thigh = link_make_cylinder(name, hy, THIGH_LEN, THIGH_DIA);
        thigh.origin_pos = v3(0, 0, -THIGH_LEN / 2);
        thigh.geom_offset= v3(0, THIGH_LEN / 2, 0);  /* center at joint */
        thigh.mass       = THIGH_MASS;
        int t = robot_add_link(r, thigh);

        /* Knee - 1 DOF: Y (pitch) */
        snprintf(name, sizeof name, "%s_knee", leg->prefix);
        Link knee = link_make_cylinder(name, t, SHIN_LEN + 0.02, 0.08);
        knee.joint      = JOINT_REVOLUTE;
        knee.origin_pos = v3(0, 0, -THIGH_LEN);
        knee.axis       = v3(0, 1, 0);  /* Y-axis for flexion/extension */
        knee.act_mode   = ACT_POSITION;
        knee.limit_min  = -0.3; knee.limit_max = 2.6;   /* wider range, can hyperextend slightly */
        knee.max_vel    = MAX_VEL_KNEE;
        knee.max_torque = MAX_TQ_KNEE;
        knee.mass       = SHIN_MASS / 2;
        int k = robot_add_link(r, knee);

        /* Shin (cylinder) */
        snprintf(name, sizeof name, "%s_shin", leg->prefix);
        Link shin = link_make_cylinder(name, k, SHIN_LEN, SHIN_DIA);
        shin.origin_pos = v3(0, 0, -SHIN_LEN / 2);
        shin.geom_offset= v3(0, SHIN_LEN / 2, 0);
        shin.mass       = SHIN_MASS;
        int sh = robot_add_link(r, shin);

        /* Ankle - 2 DOF: Y (pitch) and X (roll) */
        snprintf(name, sizeof name, "%s_ankle", leg->prefix);
        Link ankle_pitch = link_make(name, sh);
        ankle_pitch.joint      = JOINT_REVOLUTE;
        ankle_pitch.origin_pos = v3(0, 0, -SHIN_LEN);
        ankle_pitch.axis       = v3(0, 1, 0);  /* Y-axis for pitch */
        ankle_pitch.act_mode   = ACT_POSITION;
        ankle_pitch.limit_min  = -0.6; ankle_pitch.limit_max = 0.8;
        ankle_pitch.max_vel    = MAX_VEL_ANKLE;
        ankle_pitch.max_torque = MAX_TQ_ANKLE;
        int ap = robot_add_link(r, ankle_pitch);

        /* Ankle roll (X axis) */
        snprintf(name, sizeof name, "%s_ankle_roll", leg->prefix);
        Link ankle_roll = link_make(name, ap);
        ankle_roll.joint      = JOINT_REVOLUTE;
        ankle_roll.origin_pos = v3(0, 0, 0);
        ankle_roll.axis       = v3(1, 0, 0);  /* X-axis for roll */
        ankle_roll.act_mode   = ACT_POSITION;
        ankle_roll.limit_min  = -0.4; ankle_roll.limit_max = 0.4;
        ankle_roll.max_vel    = MAX_VEL_ANKLE * 0.7;
        ankle_roll.max_torque = MAX_TQ_ANKLE * 0.6;
        int ar = robot_add_link(r, ankle_roll);

        /* Foot (platform) */
        snprintf(name, sizeof name, "%s_foot", leg->prefix);
        Link foot = link_make(name, ar);
        foot.geom       = GEOM_BOX;
        foot.dims       = v3(FOOT_LEN, FOOT_W, FOOT_H);
        foot.geom_offset= v3(0, 0, -FOOT_H / 2);  /* origin at ankle */
        foot.mass       = FOOT_MASS / 2;
        robot_add_link(r, foot);
    }

    /* --- ARMS --- */
    for (int i = 0; i < 2; i++) {
        const ArmDef *arm = &ARMS[i];
        char name[ROBOT_NAME_LEN];

        /* Shoulder - 3 DOF: Y (pitch), Z (yaw), X (roll) */
        snprintf(name, sizeof name, "%s_shoulder", arm->prefix);
        Link shoulder = link_make_cylinder(name, b, 0.15, 0.10);
        shoulder.joint      = JOINT_REVOLUTE;
        shoulder.origin_pos = v3(arm->sx * 0.12, -SHOULDER_Y + TORSO_H/2, 0);
        shoulder.axis       = v3(0, 1, 0);  /* Y-axis for pitch */
        shoulder.act_mode   = ACT_POSITION;
        shoulder.limit_min  = -2.0; shoulder.limit_max = 1.5;
        shoulder.max_vel    = MAX_VEL_SHOULDER;
        shoulder.max_torque = MAX_TQ_SHOULDER;
        shoulder.mass       = ARM_UPPER_MASS / 2;
        int s = robot_add_link(r, shoulder);

        /* Shoulder roll (X axis) */
        snprintf(name, sizeof name, "%s_shoulder_roll", arm->prefix);
        Link shoulder_roll = link_make(name, s);
        shoulder_roll.joint      = JOINT_REVOLUTE;
        shoulder_roll.origin_pos = v3(0, 0, 0);
        shoulder_roll.axis       = v3(1, 0, 0);  /* X-axis for roll */
        shoulder_roll.act_mode   = ACT_POSITION;
        shoulder_roll.limit_min  = -0.5; shoulder_roll.limit_max = 2.5;
        shoulder_roll.max_vel    = MAX_VEL_SHOULDER * 0.8;
        shoulder_roll.max_torque = MAX_TQ_SHOULDER * 0.7;
        int sr = robot_add_link(r, shoulder_roll);

        /* Shoulder yaw (Z axis) */
        snprintf(name, sizeof name, "%s_shoulder_yaw", arm->prefix);
        Link shoulder_yaw = link_make(name, sr);
        shoulder_yaw.joint      = JOINT_REVOLUTE;
        shoulder_yaw.origin_pos = v3(0, 0, 0);
        shoulder_yaw.axis       = v3(0, 0, 1);  /* Z-axis for yaw */
        shoulder_yaw.act_mode   = ACT_POSITION;
        shoulder_yaw.limit_min  = -2.5; shoulder_yaw.limit_max = 0.5;
        shoulder_yaw.max_vel    = MAX_VEL_SHOULDER * 0.6;
        shoulder_yaw.max_torque = MAX_TQ_SHOULDER * 0.5;
        int sy = robot_add_link(r, shoulder_yaw);

        /* Upper arm (cylinder) */
        snprintf(name, sizeof name, "%s_upper_arm", arm->prefix);
        Link upper_arm = link_make_cylinder(name, sy, UPPER_ARM_LEN, UPPER_ARM_DIA);
        upper_arm.origin_pos = v3(0, 0, -UPPER_ARM_LEN / 2);
        upper_arm.geom_offset= v3(0, UPPER_ARM_LEN / 2, 0);
        upper_arm.mass       = ARM_UPPER_MASS;
        int ua = robot_add_link(r, upper_arm);

        /* Elbow - 1 DOF: Y (pitch) */
        snprintf(name, sizeof name, "%s_elbow", arm->prefix);
        Link elbow = link_make_cylinder(name, ua, FOREARM_LEN + 0.02, 0.06);
        elbow.joint      = JOINT_REVOLUTE;
        elbow.origin_pos = v3(0, 0, -UPPER_ARM_LEN);
        elbow.axis       = v3(0, 1, 0);  /* Y-axis for flexion */
        elbow.act_mode   = ACT_POSITION;
        elbow.limit_min  = -0.2; elbow.limit_max = 3.0;  /* wide range */
        elbow.max_vel    = MAX_VEL_ELBOW;
        elbow.max_torque = MAX_TQ_ELBOW;
        elbow.mass       = ARM_FOREARM_MASS / 2;
        int e = robot_add_link(r, elbow);

        /* Forearm (cylinder) */
        snprintf(name, sizeof name, "%s_forearm", arm->prefix);
        Link forearm = link_make_cylinder(name, e, FOREARM_LEN, FOREARM_DIA);
        forearm.origin_pos = v3(0, 0, -FOREARM_LEN / 2);
        forearm.geom_offset= v3(0, FOREARM_LEN / 2, 0);
        forearm.mass       = ARM_FOREARM_MASS;
        int f = robot_add_link(r, forearm);

        /* Wrist - 2 DOF: Y (pitch) and X (roll) */
        snprintf(name, sizeof name, "%s_wrist", arm->prefix);
        Link wrist_pitch = link_make(name, f);
        wrist_pitch.joint      = JOINT_REVOLUTE;
        wrist_pitch.origin_pos = v3(0, 0, -FOREARM_LEN);
        wrist_pitch.axis       = v3(0, 1, 0);  /* Y-axis for pitch */
        wrist_pitch.act_mode   = ACT_POSITION;
        wrist_pitch.limit_min  = -0.5; wrist_pitch.limit_max = 1.0;
        wrist_pitch.max_vel    = MAX_VEL_ELBOW * 0.8;
        wrist_pitch.max_torque = MAX_TQ_ELBOW * 0.6;
        int wp = robot_add_link(r, wrist_pitch);

        /* Wrist roll (X axis) */
        snprintf(name, sizeof name, "%s_wrist_roll", arm->prefix);
        Link wrist_roll = link_make(name, wp);
        wrist_roll.joint      = JOINT_REVOLUTE;
        wrist_roll.origin_pos = v3(0, 0, 0);
        wrist_roll.axis       = v3(1, 0, 0);  /* X-axis for roll */
        wrist_roll.act_mode   = ACT_POSITION;
        wrist_roll.limit_min  = -1.0; wrist_roll.limit_max = 1.0;
        wrist_roll.max_vel    = MAX_VEL_ELBOW * 0.6;
        wrist_roll.max_torque = MAX_TQ_ELBOW * 0.5;
        robot_add_link(r, wrist_roll);
    }

    /* Set initial standing pose */
    r->base_pos = v3(0, 0, STAND_Z);

    /* Default joint positions for standing. q[] is indexed by LINK, so look
     * the links up by name rather than hardcoding indices. */
    r->q[robot_find_link(r, "l_hip")]      = PELVIS_PITCH_STABLE;
    r->q[robot_find_link(r, "r_hip")]      = PELVIS_PITCH_STABLE;
    r->q[robot_find_link(r, "l_hip_roll")] = STAND_HIP_Y;
    r->q[robot_find_link(r, "r_hip_roll")] = -STAND_HIP_Y;

    /* Knees - slightly bent for stability */
    r->q[robot_find_link(r, "l_knee")] = 0.3;
    r->q[robot_find_link(r, "r_knee")] = 0.3;

    /* Arms - raised slightly for balance */
    r->q[robot_find_link(r, "l_shoulder")] = -0.5;
    r->q[robot_find_link(r, "r_shoulder")] = 0.3;

    r->base_update = robot_base_update_twist;
}

/* --- demo controller: improved human-like walk cycle ------------------- */
static void biped_ctrl_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static void biped_ctrl_step(Controller *c, const Observation *obs,
                            double *cmd, int n_cmd, double base_cmd[3]) {
    (void)c; (void)n_cmd;

    /* Simple walking controller with arm swing */
    double freq = 1.2;        /* step frequency (Hz) */
    double speed = 0.4;       /* forward speed (m/s) */
    double phase_offset = M_PI;  /* leg swing phase offset */

    double phi = 2 * M_PI * freq * obs->time;

    /* For each leg, compute target positions */
    for (int i = 0; i < 2; i++) {
        double leg_phase = phi + (i == 0 ? 0.0 : phase_offset);
        double swing = fmax(0.0, sin(leg_phase));  /* 0 during stance, >0 during swing */

        /* Hip: pitch (forward/back) with slight hip offset */
        double hip_pitch = 0.15 + 0.40 * sin(leg_phase);

        /* Hip roll: slight side-to-side for balance */
        double hip_roll = 0.05 * sin(leg_phase + M_PI/2);

        /* Knee: swing up during leg lift */
        double knee = 0.15 + 0.80 * swing;

        /* Ankle: pitch for foot clearance and stability */
        double ankle = -(hip_pitch - knee) * 0.6;

        int lb = BIP_LEG_BASE(i);
        cmd[lb + BIP_HIP_PITCH]   = clampd(hip_pitch, -1.8, 1.8);
        cmd[lb + BIP_HIP_ROLL]    = clampd(hip_roll, -0.8, 0.8);
        cmd[lb + BIP_HIP_YAW]     = 0.0;
        cmd[lb + BIP_KNEE]        = clampd(knee, -0.3, 2.6);
        cmd[lb + BIP_ANKLE_PITCH] = clampd(ankle, -0.6, 0.8);
        cmd[lb + BIP_ANKLE_ROLL]  = 0.0;
    }

    /* Arm swing for balance (opposite to legs) */
    for (int i = 0; i < 2; i++) {
        double arm_phase = phi + (i == 0 ? M_PI : 0.0);  /* opposite to legs */
        double swing = fmax(0.0, sin(arm_phase));

        /* Shoulder pitch */
        double shoulder = -0.3 + 0.5 * sin(arm_phase);

        /* Elbow flexion */
        double elbow = 0.2 + 0.6 * swing;

        /* Wrist */
        double wrist = -(shoulder - elbow) * 0.5;

        int ab = BIP_ARM_BASE(i);
        cmd[ab + BIP_SH_PITCH]    = clampd(shoulder, -2.0, 1.5);
        cmd[ab + BIP_SH_ROLL]     = 0.0;
        cmd[ab + BIP_SH_YAW]      = 0.0;
        cmd[ab + BIP_ELBOW]       = clampd(elbow, -0.2, 3.0);
        cmd[ab + BIP_WRIST_PITCH] = clampd(wrist, -0.5, 1.0);
        cmd[ab + BIP_WRIST_ROLL]  = 0.0;
    }

    /* Base twist for forward motion */
    base_cmd[0] = speed;              /* forward velocity */
    base_cmd[1] = 0.0;                /* lateral velocity */
    base_cmd[2] = 0.0;                /* no turning for now */
}

Controller *biped_demo_controller(void) {
    static Controller c = {
        .name  = "humanoid_walk",
        .reset = biped_ctrl_reset,
        .step  = biped_ctrl_step,
    };
    return &c;
}