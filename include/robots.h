/* robots.h — the registry of available robot models.
 *
 * Each robot model lives in its own file under robots/ and provides:
 *   build()            — assemble the actuator tree (links, dims, limits)
 *                        and set the initial base pose + base_update callback
 *   demo_controller()  — a simple built-in controller that makes it move
 *
 * To add a new robot: write robots/<name>.c with those two functions and add
 * one row to ROBOT_MODELS in robots/registry.c.
 */
#ifndef ROBOTS_H
#define ROBOTS_H

#include "world.h"

typedef struct {
    const char *name;
    const char *description;
    void       (*build)(Robot *r);
    Controller *(*demo_controller)(void);
} RobotModel;

extern const RobotModel ROBOT_MODELS[];
extern const int        N_ROBOT_MODELS;

const RobotModel *robot_model_find(const char *name);

/* Build (or rebuild) the given model into the world and reset. */
void world_load_model(World *w, const RobotModel *m);

#endif /* ROBOTS_H */
