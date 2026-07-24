/* registry.c — the table of available robot models.
 * Adding a robot: write robots/<name>.c, declare it here, add one row.
 */
#include "robots.h"
#include <string.h>

void car_build(Robot *r);
void quadruped_build(Robot *r);
void biped_build(Robot *r);
Controller *car_demo_controller(void);
Controller *quadruped_demo_controller(void);
Controller *biped_demo_controller(void);

const RobotModel ROBOT_MODELS[] = {
    {"car",       "4-wheel car, Ackermann steering (6 actuators)",
     car_build,       car_demo_controller},
    {"quadruped", "4-legged trotting robot (8 actuators)",
     quadruped_build, quadruped_demo_controller},
    {"biped",     "2-legged walking robot (6 actuators)",
     biped_build,     biped_demo_controller},
};
const int N_ROBOT_MODELS = (int)(sizeof ROBOT_MODELS / sizeof ROBOT_MODELS[0]);

const RobotModel *robot_model_find(const char *name) {
    for (int i = 0; i < N_ROBOT_MODELS; i++)
        if (strcmp(ROBOT_MODELS[i].name, name) == 0)
            return &ROBOT_MODELS[i];
    return NULL;
}

void world_load_model(World *w, const RobotModel *m) {
    m->build(&w->robot);
    w->controller = m->demo_controller();
    world_reset(w);
}
