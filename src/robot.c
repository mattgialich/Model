#include "robot.h"
#include <string.h>

Link link_make(const char *name, int parent) {
    Link L;
    memset(&L, 0, sizeof L);
    strncpy(L.name, name, ROBOT_NAME_LEN - 1);
    L.parent     = parent;
    L.joint      = JOINT_FIXED;
    L.origin_rot = quat_identity();
    L.axis       = v3(0, 0, 1);
    L.act_mode   = ACT_NONE;
    L.limit_min  = -1e9;
    L.limit_max  =  1e9;
    L.max_vel    =  1e9;
    L.gear_ratio =  1.0;
    L.geom       = GEOM_BOX;
    L.dims       = v3(0.05, 0.05, 0.05);
    L.mass       = 1.0;
    L.inertia    = v3(1e-3, 1e-3, 1e-3);
    return L;
}

void robot_init(Robot *r) {
    memset(r, 0, sizeof *r);
    r->base_rot = quat_identity();
}

int robot_add_link(Robot *r, Link link) {
    int i = r->n_links++;
    r->links[i] = link;
    if (link.act_mode != ACT_NONE)
        r->act_link[r->n_actuators++] = i;
    return i;
}

int robot_find_link(const Robot *r, const char *name) {
    for (int i = 0; i < r->n_links; i++)
        if (strncmp(r->links[i].name, name, ROBOT_NAME_LEN) == 0)
            return i;
    return -1;
}

void robot_step(Robot *r, double dt) {
    for (int a = 0; a < r->n_actuators; a++) {
        int i = r->act_link[a];
        const Link *L = &r->links[i];
        double c = r->cmd[a];
        double v;

        if (L->act_mode == ACT_VELOCITY) {
            v = clampd(c, -L->max_vel, L->max_vel);
        } else { /* ACT_POSITION: chase the target at up to max_vel */
            double target = clampd(c, L->limit_min, L->limit_max);
            v = clampd((target - r->q[i]) / dt, -L->max_vel, L->max_vel);
        }

        r->dq[i] = v;
        r->q[i] += v * dt;

        if (L->joint == JOINT_WHEEL) {
            /* keep the angle bounded for numerical hygiene */
            if (r->q[i] > 2 * M_PI || r->q[i] < -2 * M_PI)
                r->q[i] = fmod(r->q[i], 2 * M_PI);
        } else {
            double clamped = clampd(r->q[i], L->limit_min, L->limit_max);
            if (clamped != r->q[i]) { r->q[i] = clamped; r->dq[i] = 0; }
        }
    }

    if (r->base_update)
        r->base_update(r, dt);

    robot_fk(r);
}

void robot_fk(Robot *r) {
    for (int i = 0; i < r->n_links; i++) {
        const Link *L = &r->links[i];
        Vec3 pp;
        Quat pr;
        if (L->parent < 0) { pp = r->base_pos;             pr = r->base_rot; }
        else               { pp = r->link_pos[L->parent];  pr = r->link_rot[L->parent]; }

        Vec3 pos   = v3_add(pp, quat_rotate(pr, L->origin_pos));
        Quat frame = quat_mul(pr, L->origin_rot);

        switch (L->joint) {
        case JOINT_REVOLUTE:
        case JOINT_WHEEL:
            frame = quat_mul(frame, quat_axis_angle(L->axis, r->q[i]));
            break;
        case JOINT_PRISMATIC:
            pos = v3_add(pos, quat_rotate(frame, v3_scale(L->axis, r->q[i])));
            break;
        case JOINT_FIXED:
            break;
        }

        r->link_pos[i] = pos;
        r->link_rot[i] = quat_normalize(frame);
    }
}

void robot_base_update_twist(Robot *r, double dt) {
    double vx = r->base_cmd[0], vy = r->base_cmd[1], wz = r->base_cmd[2];

    r->base_rot = quat_normalize(
        quat_mul(quat_axis_angle(v3(0, 0, 1), wz * dt), r->base_rot));

    Vec3 fwd  = quat_rotate(r->base_rot, v3(1, 0, 0));
    Vec3 left = quat_rotate(r->base_rot, v3(0, 1, 0));
    Vec3 step = v3_add(v3_scale(v3(fwd.x,  fwd.y,  0), vx * dt),
                       v3_scale(v3(left.x, left.y, 0), vy * dt));
    r->base_pos = v3_add(r->base_pos, step);
}
