/* main.c — headless entry point. Runs the sim at full speed with no
 * graphics and reports state + throughput. This is the mode an intelligence
 * model would train against.
 * Bionic - AI Assistant
 *
 * Usage: ./model [robot] [seconds]
 *        ./model quadruped 30
 */
#include "robots.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

/* Monotonic wall clock in integer nanoseconds — immune to system clock
 * adjustments and free of floating-point accumulation error. */
static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void print_usage(void) {
    printf("usage: model [robot] [seconds]\n\navailable robots:\n");
    for (int i = 0; i < N_ROBOT_MODELS; i++)
        printf("  %-10s %s\n", ROBOT_MODELS[i].name,
               ROBOT_MODELS[i].description);
}

int main(int argc, char **argv) {
    const char *name    = argc > 1 ? argv[1] : "car";
    double sim_seconds  = argc > 2 ? atof(argv[2]) : 10.0;

    const RobotModel *m = robot_model_find(name);
    if (!m) {
        fprintf(stderr, "unknown robot '%s'\n\n", name);
        print_usage();
        return 1;
    }

    World w;
    world_init(&w, 0.001);          /* 1 kHz physics */
    world_load_model(&w, m);

    printf("robot: %s  (%d links, %d actuators)  dt=%.4fs\n",
           w.robot.name, w.robot.n_links, w.robot.n_actuators, w.dt);

    long total_steps = (long)(sim_seconds / w.dt);
    long print_every = (long)(1.0 / w.dt);

    /* Time the stepping in chunks so the per-second status printfs stay
     * outside the measured interval — throughput reflects the sim alone. */
    uint64_t sim_ns = 0;
    uint64_t t_start = now_ns();

    for (long done = 0; done < total_steps; ) {
        long chunk = print_every < total_steps - done
                   ? print_every : total_steps - done;

        uint64_t c0 = now_ns();
        for (long s = 0; s < chunk; s++)
            world_step(&w);
        sim_ns += now_ns() - c0;
        done += chunk;

        if (chunk == print_every) {
            Robot *r = &w.robot;
            printf("t=%5.1fs  base pos=(%7.3f, %7.3f, %6.3f)  yaw=%6.1f deg\n",
                   w.time, r->base_pos.x, r->base_pos.y, r->base_pos.z,
                   quat_yaw(r->base_rot) * 180.0 / M_PI);
        }
    }

    uint64_t total_ns = now_ns() - t_start;

    printf("\n%ld steps, stepping time %.6fs (total wall %.6fs incl. output)\n",
           total_steps, sim_ns / 1e9, total_ns / 1e9);
    if (total_steps > 0 && sim_ns > 0)
        printf("%.2f Msteps/s  (%.0fx real time)\n",
               total_steps / (sim_ns / 1e9) / 1e6,
               (total_steps * w.dt) / (sim_ns / 1e9));
    return 0;
}
