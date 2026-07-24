/* biped_balance.h — Balance control for the biped humanoid.
 *
 * Features:
 * - ZMP (Zero Moment Point) computation
 * - CoM (Center of Mass) tracking
 * - Balance feedback correction
 */
#ifndef BIPED_BALANCE_H
#define BIPED_BALANCE_H

#include "robot.h"
#include "control.h"   /* Observation, Controller */

/* Actuator index layout produced by biped_build (add order).
 * Legs first (left, right), then arms (left, right), 6 actuators each. */
#define BIP_LEG_BASE(i)  ((i) * 6)
#define BIP_ARM_BASE(i)  (12 + (i) * 6)

enum {
    BIP_HIP_PITCH = 0, BIP_HIP_ROLL, BIP_HIP_YAW,
    BIP_KNEE, BIP_ANKLE_PITCH, BIP_ANKLE_ROLL,
};
enum {
    BIP_SH_PITCH = 0, BIP_SH_ROLL, BIP_SH_YAW,
    BIP_ELBOW, BIP_WRIST_PITCH, BIP_WRIST_ROLL,
};

/* Balance state tracking */
typedef struct {
    double last_com_x;
    double last_com_y;
    double zmp_error_sum;
} BalanceState;

extern BalanceState biped_balance_state;

/* Compute center of mass from robot state */
Vec3 biped_compute_com(const Robot *r);

/* Compute Zero Moment Point for stability assessment */
Vec3 biped_compute_zmp(const Robot *r, const Observation *obs);

/* Apply balance correction to joint commands */
void biped_apply_balance_correction(double *cmd, int n_cmd,
                                     const Observation *obs);

#endif /* BIPED_BALANCE_H */
