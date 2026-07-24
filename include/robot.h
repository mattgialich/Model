/* robot.h — a robot is a tree of links connected by joints (the actuator tree).
 *
 * Every link stores its design properties: geometry/dimensions, joint type and
 * axis, actuator limits (position, velocity, torque, gear ratio), and mass /
 * inertia. Mass and inertia are unused by the kinematic stepper but are carried
 * so full rigid-body dynamics can be added later without changing any robot
 * definition.
 *
 * The robot base is a floating 6-DOF pose (base_pos / base_rot). How the base
 * moves is model-specific — a car derives it from wheel speed and steering, a
 * legged robot from its gait — so each model supplies a base_update callback.
 */
#ifndef ROBOT_H
#define ROBOT_H

#include "math3d.h"

#define ROBOT_MAX_LINKS 64
#define ROBOT_NAME_LEN  32

typedef enum {
    JOINT_FIXED,      /* rigid connection, no state */
    JOINT_REVOLUTE,   /* rotation about axis, limited */
    JOINT_PRISMATIC,  /* translation along axis, limited */
    JOINT_WHEEL       /* continuous rotation about axis (no position limits) */
} JointType;

typedef enum {
    GEOM_BOX,         /* dims = (size x, size y, size z) */
    GEOM_CYLINDER,    /* dims = (radius, length along local Y, -) */
    GEOM_SPHERE       /* dims = (radius, -, -) */
} GeomType;

typedef enum {
    ACT_NONE,         /* passive / fixed link, not actuated */
    ACT_POSITION,     /* command is a target position (rad or m) */
    ACT_VELOCITY      /* command is a target velocity (rad/s or m/s) */
} ActuatorMode;

typedef struct {
    char      name[ROBOT_NAME_LEN];
    int       parent;          /* index of parent link, -1 = attached to base */

    /* --- joint / actuator design properties --- */
    JointType joint;
    Vec3      origin_pos;      /* joint origin, expressed in parent frame */
    Quat      origin_rot;      /* joint frame orientation in parent frame */
    Vec3      axis;            /* joint axis in this link's frame (unit) */
    ActuatorMode act_mode;
    double    limit_min, limit_max;  /* position limits (rad or m) */
    double    max_vel;         /* actuator speed limit (rad/s or m/s) */
    double    max_torque;      /* stored for future dynamics */
    double    gear_ratio;      /* stored for future dynamics */

    /* --- geometry (visual + future collision) --- */
    GeomType  geom;
    Vec3      dims;            /* meaning depends on geom, see enum above */
    Vec3      geom_offset;     /* geometry center in this link's frame */

    /* --- inertial (unused by kinematics, ready for dynamics) --- */
    double    mass;            /* kg */
    Vec3      com;             /* center of mass in link frame */
    Vec3      inertia;         /* diagonal inertia about com (kg m^2) */
} Link;

typedef struct Robot Robot;
struct Robot {
    char name[ROBOT_NAME_LEN];

    int  n_links;
    Link links[ROBOT_MAX_LINKS];

    /* Actuator table: actuator i drives links[act_link[i]].
     * Built automatically as actuated links are added (in add order). */
    int  n_actuators;
    int  act_link[ROBOT_MAX_LINKS];

    /* --- state --- */
    double q[ROBOT_MAX_LINKS];    /* joint position per link (0 for fixed) */
    double dq[ROBOT_MAX_LINKS];   /* joint velocity per link */
    double cmd[ROBOT_MAX_LINKS];  /* latest command per ACTUATOR index */

    Vec3   base_pos;              /* floating base pose in world frame */
    Quat   base_rot;
    double base_cmd[3];           /* commanded body twist: vx, vy, yaw rate.
                                     Used by base_update of legged models;
                                     the car ignores it (derives motion from
                                     its wheels instead). */

    /* --- forward kinematics output (world frame, filled by robot_fk) --- */
    Vec3 link_pos[ROBOT_MAX_LINKS];
    Quat link_rot[ROBOT_MAX_LINKS];

    /* Model-specific base propagation. May be NULL (base stays put). */
    void (*base_update)(Robot *r, double dt);
};

/* A Link pre-filled with sane defaults (identity origin, z axis, unlimited,
 * small box geometry). Set only the fields you care about. */
Link link_make(const char *name, int parent);

void robot_init(Robot *r);                 /* zero everything, identity base */
int  robot_add_link(Robot *r, Link link);  /* returns link index */

/* Advance joints one timestep from r->cmd, run base_update, then FK. */
void robot_step(Robot *r, double dt);

/* Recompute world-frame link transforms from base pose + joint positions. */
void robot_fk(Robot *r);

/* Shared base_update for legged models: integrates base_cmd (body-frame
 * vx, vy, yaw rate) while holding base height constant. */
void robot_base_update_twist(Robot *r, double dt);

int robot_find_link(const Robot *r, const char *name);  /* -1 if missing */

#endif /* ROBOT_H */
