/* control.h — the seam between the world model and an intelligence model.
 *
 * Each tick the world builds an Observation and hands it to the controller;
 * the controller writes one command per actuator (interpreted per that
 * actuator's mode: target position or target velocity) plus an optional
 * commanded base twist for legged models.
 *
 * To connect a learned policy / intelligence model, implement Controller.step
 * to marshal the Observation into your model's input and copy its output into
 * cmd[]. Nothing else in the simulator needs to change.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "robot.h"

typedef struct {
    double       time;         /* seconds since reset */
    int          n_links;
    int          n_actuators;
    const double *q;           /* joint positions, indexed by link */
    const double *dq;          /* joint velocities, indexed by link */
    const int    *act_link;    /* actuator i -> link index */
    Vec3         base_pos;     /* world-frame base pose */
    Quat         base_rot;
} Observation;

typedef struct Controller Controller;
struct Controller {
    const char *name;
    void       *ctx;           /* controller-private state */

    /* Called when the world (re)starts. */
    void (*reset)(Controller *c, const Robot *r);

    /* Called every tick. Write cmd[0..n_cmd-1] (one entry per actuator,
     * in actuator order) and optionally base_cmd = {vx, vy, yaw_rate}. */
    void (*step)(Controller *c, const Observation *obs,
                 double *cmd, int n_cmd, double base_cmd[3]);
};

#endif /* CONTROL_H */
