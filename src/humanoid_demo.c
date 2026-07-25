/* humanoid_demo.c — Simple demo of the humanoid robot walking */
#include "robots.h"
#include <stdio.h>

int main(void) {
    World w;
    world_init(&w, 0.001);
    world_load_model(&w, robot_model_find("humanoid"));

    printf("Humanoid Demo - Walking Simulation\n");
    printf("=================================\n\n");

    /* Print initial state */
    printf("Initial position: (%.3f, %.3f, %.3f)\n",
           w.robot.base_pos.x, w.robot.base_pos.y, w.robot.base_pos.z);
    printf("Initial rotation: yaw=%.3f\n", quat_yaw(w.robot.base_rot));
    printf("\n");

    /* Run simulation */
    const int STEPS = 1200;  /* 1.2 seconds at 0.001s timestep */
    const int PRINT_INTERVAL = 200;  /* Print every 0.2 seconds */

    printf("Simulating %d steps (%.1f seconds)...\n\n", STEPS, STEPS * 0.001);

    for (int i = 0; i < STEPS; i++) {
        world_step(&w);

        if (i % PRINT_INTERVAL == 0) {
            printf("Step %4d: pos=(%.3f, %.3f, %.3f) yaw=%.3f\n",
                   i,
                   w.robot.base_pos.x,
                   w.robot.base_pos.y,
                   w.robot.base_pos.z,
                   quat_yaw(w.robot.base_rot));
        }
    }

    printf("\n");
    printf("Final position: (%.3f, %.3f, %.3f)\n",
           w.robot.base_pos.x, w.robot.base_pos.y, w.robot.base_pos.z);
    printf("Total displacement: %.3f meters forward\n", w.robot.base_pos.x);

    return 0;
}
