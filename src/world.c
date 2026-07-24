#include "world.h"
#include <string.h>

void world_init(World *w, double dt) {
    memset(w, 0, sizeof *w);
    w->dt      = dt;
    w->gravity = 9.81;
    robot_init(&w->robot);
}

void world_reset(World *w) {
    w->time = 0.0;
    memset(w->robot.cmd, 0, sizeof w->robot.cmd);
    memset(w->robot.base_cmd, 0, sizeof w->robot.base_cmd);
    if (w->controller && w->controller->reset)
        w->controller->reset(w->controller, &w->robot);
    robot_fk(&w->robot);
}

void world_step(World *w) {
    Robot *r = &w->robot;

    if (w->controller) {
        Observation obs = {
            .time        = w->time,
            .n_links     = r->n_links,
            .n_actuators = r->n_actuators,
            .q           = r->q,
            .dq          = r->dq,
            .act_link    = r->act_link,
            .base_pos    = r->base_pos,
            .base_rot    = r->base_rot,
        };
        w->controller->step(w->controller, &obs, r->cmd, r->n_actuators,
                            r->base_cmd);
    }

    robot_step(r, w->dt);
    w->time += w->dt;
}
