/* car.c — four-wheel car with Ackermann-style front steering.
 *
 * Actuator tree:
 *   chassis (root)
 *     ├─ steer_fl (revolute Z, position)  ── wheel_fl (wheel Y, velocity)
 *     ├─ steer_fr (revolute Z, position)  ── wheel_fr (wheel Y, velocity)
 *     ├─ wheel_rl (wheel Y, velocity)
 *     └─ wheel_rr (wheel Y, velocity)
 *
 * Base propagation: kinematic bicycle model — forward speed comes from the
 * rear wheel spin rate * wheel radius, yaw rate from v/L * tan(steer).
 */
#include "robots.h"
#include <string.h>

/* --- design parameters ------------------------------------------------- */
static const double WHEELBASE = 0.32;   /* front-to-rear axle, m */
static const double TRACK     = 0.26;   /* left-to-right wheel, m */
static const double WHEEL_R   = 0.06;   /* wheel radius, m */
static const double WHEEL_W   = 0.04;   /* wheel width, m */
static const double MAX_STEER = 0.55;   /* rad */
static const double MAX_WSPD  = 40.0;   /* wheel rad/s (~2.4 m/s) */

/* link indices resolved at build time, used by base_update */
static int li_steer_l, li_steer_r, li_wheel_rl, li_wheel_rr;

/* actuator indices (add order) */
enum { A_STEER_FL, A_WHEEL_FL, A_STEER_FR, A_WHEEL_FR, A_WHEEL_RL, A_WHEEL_RR };

static void car_base_update(Robot *r, double dt) {
    double steer = 0.5 * (r->q[li_steer_l] + r->q[li_steer_r]);
    double v     = 0.5 * (r->dq[li_wheel_rl] + r->dq[li_wheel_rr]) * WHEEL_R;

    double yaw_rate = v / WHEELBASE * tan(steer);
    r->base_rot = quat_normalize(
        quat_mul(quat_axis_angle(v3(0, 0, 1), yaw_rate * dt), r->base_rot));

    Vec3 fwd = quat_rotate(r->base_rot, v3(1, 0, 0));
    r->base_pos = v3_add(r->base_pos, v3_scale(v3(fwd.x, fwd.y, 0), v * dt));
}

static int add_wheel(Robot *r, const char *name, int parent, Vec3 origin) {
    Link w = link_make(name, parent);
    w.joint      = JOINT_WHEEL;
    w.origin_pos = origin;
    w.axis       = v3(0, 1, 0);
    w.act_mode   = ACT_VELOCITY;
    w.max_vel    = MAX_WSPD;
    w.max_torque = 2.0;
    w.geom       = GEOM_CYLINDER;
    w.dims       = v3(WHEEL_R, WHEEL_W, 0);
    w.mass       = 0.3;
    return robot_add_link(r, w);
}

void car_build(Robot *r) {
    robot_init(r);
    strncpy(r->name, "car", ROBOT_NAME_LEN - 1);

    Link chassis = link_make("chassis", -1);
    chassis.geom        = GEOM_BOX;
    chassis.dims        = v3(0.44, 0.20, 0.08);
    chassis.geom_offset = v3(0, 0, 0.06);
    chassis.mass        = 3.0;
    int body = robot_add_link(r, chassis);

    double fx = WHEELBASE / 2, hy = TRACK / 2;

    /* front left: steering knuckle, then wheel hanging off it */
    Link sl = link_make("steer_fl", body);
    sl.joint = JOINT_REVOLUTE; sl.origin_pos = v3(fx, hy, 0);
    sl.axis = v3(0, 0, 1); sl.act_mode = ACT_POSITION;
    sl.limit_min = -MAX_STEER; sl.limit_max = MAX_STEER;
    sl.max_vel = 6.0; sl.max_torque = 1.5;
    sl.geom = GEOM_SPHERE; sl.dims = v3(0.015, 0, 0);
    li_steer_l = robot_add_link(r, sl);
    add_wheel(r, "wheel_fl", li_steer_l, v3(0, 0.03, 0));

    Link sr = sl;
    strncpy(sr.name, "steer_fr", ROBOT_NAME_LEN - 1);
    sr.origin_pos = v3(fx, -hy, 0);
    li_steer_r = robot_add_link(r, sr);
    add_wheel(r, "wheel_fr", li_steer_r, v3(0, -0.03, 0));

    li_wheel_rl = add_wheel(r, "wheel_rl", body, v3(-fx,  hy + 0.03, 0));
    li_wheel_rr = add_wheel(r, "wheel_rr", body, v3(-fx, -hy - 0.03, 0));

    r->base_pos    = v3(0, 0, WHEEL_R);   /* axle height above the ground */
    r->base_update = car_base_update;
}

/* --- demo controller: gentle S-curves at constant speed ----------------- */
static void car_ctrl_reset(Controller *c, const Robot *r) {
    (void)c; (void)r;
}

static void car_ctrl_step(Controller *c, const Observation *obs,
                          double *cmd, int n_cmd, double base_cmd[3]) {
    (void)c; (void)n_cmd; (void)base_cmd;
    double steer = 0.35 * sin(0.5 * obs->time);
    double wspd  = 1.2 / WHEEL_R;          /* 1.2 m/s forward */

    cmd[A_STEER_FL] = steer;
    cmd[A_STEER_FR] = steer;
    cmd[A_WHEEL_FL] = wspd;
    cmd[A_WHEEL_FR] = wspd;
    cmd[A_WHEEL_RL] = wspd;
    cmd[A_WHEEL_RR] = wspd;
}

Controller *car_demo_controller(void) {
    static Controller c = {
        .name  = "car_demo",
        .reset = car_ctrl_reset,
        .step  = car_ctrl_step,
    };
    return &c;
}
