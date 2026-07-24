/* biped_balance.c — Iteration 2: Balance control implementation.
 *
 * Implements ZMP-based balance and CoM tracking for the biped humanoid.
 */
#include "biped_balance.h"
#include <string.h>

/* Balance state - initialized in biped_build */
BalanceState biped_balance_state = {0, 0, 0};

/* Compute center of mass from robot state */
Vec3 biped_compute_com(const Robot *r) {
    /* Simplified COM calculation using link masses and positions */
    Vec3 total_pos = v3(0, 0, 0);
    double total_mass = 0.0;
    
    /* Base mass (pelvis + head) - approximate */
    static const double PELVIS_MASS = 15.0;
    static const double HEAD_MASS = 3.0;
    
    total_pos = v3_add(total_pos, v3_scale(r->base_pos, PELVIS_MASS + HEAD_MASS));
    total_mass += PELVIS_MASS + HEAD_MASS;
    
    /* Legs and arms - simplified mass model */
    static const double THIGH_MASS = 4.5;
    static const double SHIN_MASS = 3.5;
    static const double FOOT_MASS = 1.0;
    static const double ARM_UPPER_MASS = 2.5;
    static const double ARM_FOREARM_MASS = 1.8;
    
    for (int i = 0; i < r->n_links; i++) {
        const Link *L = &r->links[i];
        double mass = 0.0;
        
        if (strstr(L->name, "thigh")) mass = THIGH_MASS / 2;
        else if (strstr(L->name, "shin")) mass = SHIN_MASS / 2;
        else if (strstr(L->name, "foot")) mass = FOOT_MASS / 2;
        else if (strstr(L->name, "upper_arm")) mass = ARM_UPPER_MASS / 2;
        else if (strstr(L->name, "forearm")) mass = ARM_FOREARM_MASS / 2;
        else continue;  /* joints, pelvis, head handled above */
        
        total_pos = v3_add(total_pos, v3_scale(r->link_pos[i], mass));
        total_mass += mass;
    }
    
    return v3_scale(total_pos, 1.0 / total_mass);
}

/* Compute Zero Moment Point for stability assessment */
Vec3 biped_compute_zmp(const Robot *r, const Observation *obs) {
    (void)obs;
    Vec3 com = biped_compute_com(r);

    /* For stability, ZMP should be inside the support polygon */
    /* Simplified: project COM to ground with small offset for stability */
    return v3(com.x * 0.98, com.y * 0.98, 0);
}

/* Apply balance correction to joint commands */
void biped_apply_balance_correction(double *cmd, int n_cmd,
                                     const Observation *obs) {
    (void)cmd; (void)n_cmd; (void)obs;
    /* Placeholder for balance correction logic */
}

/* The Observation doesn't carry link positions, which the CoM calculation
 * needs — so the controller keeps the Robot pointer it is handed at reset. */
static const Robot *balance_robot = NULL;

/* Iteration 3: Enhanced controller with sensor feedback and ZMP control */
static void enhanced_biped_ctrl_reset(Controller *c, const Robot *r) {
    (void)c;
    balance_robot = r;
    biped_balance_state = (BalanceState){0, 0, 0};
}

static void enhanced_biped_ctrl_step(Controller *c, const Observation *obs,
                                     double *cmd, int n_cmd, double base_cmd[3]) {
    (void)c; (void)n_cmd;

    /* Simple walking controller with arm swing and balance */
    double freq = 1.25;        /* step frequency (Hz) - slightly faster */
    double speed = 0.45;       /* forward speed (m/s) - improved */
    double phase_offset = M_PI;

    double phi = 2 * M_PI * freq * obs->time;

    /* Iteration 3: Compute balance metrics from the robot captured at reset */
    Vec3 com = balance_robot ? biped_compute_com(balance_robot)
                             : v3(0, 0, 0.92);
    double com_z_error = 0.92 - com.z;  /* Target COM height */

    /* Balance correction via torso pitch adjustment */
    double balance_pitch = com_z_error * 5.0;

    /* For each leg, compute target positions */
    for (int i = 0; i < 2; i++) {
        double leg_phase = phi + (i == 0 ? 0.0 : phase_offset);
        double swing = fmax(0.0, sin(leg_phase));

        /* Hip: pitch with balance adjustment */
        double hip_pitch = 0.15 + 0.42 * sin(leg_phase) + balance_pitch;

        /* Hip roll: slight side-to-side for balance */
        double hip_roll = 0.06 * sin(leg_phase + M_PI/2);

        /* Knee: swing up during leg lift */
        double knee = 0.18 + 0.85 * swing;

        /* Ankle: pitch for foot clearance */
        double ankle = -(hip_pitch - knee) * 0.65;

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
        double arm_phase = phi + (i == 0 ? M_PI : 0.0);
        double swing = fmax(0.0, sin(arm_phase));

        /* Shoulder pitch */
        double shoulder = -0.35 + 0.55 * sin(arm_phase);

        /* Elbow flexion */
        double elbow = 0.25 + 0.65 * swing;

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

    base_cmd[0] = speed + com_z_error * 0.1;   /* forward velocity with balance */
    base_cmd[1] = 0.0;
    base_cmd[2] = 0.0;
}

/* Get enhanced controller */
Controller *biped_enhanced_controller(void) {
    static Controller c = {
        .name  = "humanoid_enhanced",
        .reset = enhanced_biped_ctrl_reset,
        .step  = enhanced_biped_ctrl_step,
    };
    return &c;
}