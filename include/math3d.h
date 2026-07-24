/* math3d.h — minimal 3D math for the world model.
 * Doubles throughout. Z is up. All functions are static inline for speed.
 */
#ifndef MATH3D_H
#define MATH3D_H

#include <math.h>

typedef struct { double x, y, z; } Vec3;
typedef struct { double w, x, y, z; } Quat;   /* unit quaternion, w first */

static inline Vec3 v3(double x, double y, double z) { return (Vec3){x, y, z}; }
static inline Vec3 v3_add(Vec3 a, Vec3 b)   { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vec3 v3_sub(Vec3 a, Vec3 b)   { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline Vec3 v3_scale(Vec3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline double v3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
    return v3(a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}
static inline double v3_len(Vec3 a) { return sqrt(v3_dot(a, a)); }

static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline Quat quat_identity(void) { return (Quat){1, 0, 0, 0}; }

static inline Quat quat_mul(Quat a, Quat b) {
    return (Quat){
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

static inline Quat quat_axis_angle(Vec3 axis, double angle) {
    double h = 0.5 * angle, s = sin(h);
    return (Quat){cos(h), axis.x * s, axis.y * s, axis.z * s};
}

static inline Quat quat_normalize(Quat q) {
    double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) return quat_identity();
    double inv = 1.0 / n;
    return (Quat){q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

/* Rotate vector v by quaternion q:  v' = v + 2w(u x v) + 2 u x (u x v) */
static inline Vec3 quat_rotate(Quat q, Vec3 v) {
    Vec3 u = v3(q.x, q.y, q.z);
    Vec3 t = v3_scale(v3_cross(u, v), 2.0);
    return v3_add(v3_add(v, v3_scale(t, q.w)), v3_cross(u, t));
}

/* Yaw (rotation about world Z) extracted from a quaternion. */
static inline double quat_yaw(Quat q) {
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

#endif /* MATH3D_H */
