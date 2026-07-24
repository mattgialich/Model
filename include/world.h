/* world.h — the world: a flat ground plane at z = 0, gravity, a fixed
 * timestep, one robot, and the controller driving it.
 */
#ifndef WORLD_H
#define WORLD_H

#include "robot.h"
#include "control.h"

typedef struct {
    double     time;       /* sim time, seconds */
    double     dt;         /* fixed timestep, seconds */
    double     gravity;    /* m/s^2, stored for future dynamics */
    Robot      robot;
    Controller *controller;
} World;

void world_init(World *w, double dt);

/* Zero sim time and re-run the controller's reset. (Robot state itself is
 * reset by rebuilding the model — see world_load_model in robots.h.) */
void world_reset(World *w);

/* One tick: observe -> control -> step joints -> propagate base -> FK. */
void world_step(World *w);

#endif /* WORLD_H */
