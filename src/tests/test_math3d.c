/* test_math3d.c — Unit tests for math3d.h functions */
#include "math3d.h"
#include "test_runner.h"
#include <stdio.h>
#include <math.h>

/* Test Vec3 operations */
TEST(math3d_v3_creation) {
    Vec3 v = v3(1.0, 2.0, 3.0);
    ASSERT_FLOAT_EQ(v.x, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(v.y, 2.0, 1e-9);
    ASSERT_FLOAT_EQ(v.z, 3.0, 1e-9);
}

TEST(math3d_v3_add) {
    Vec3 a = v3(1.0, 2.0, 3.0);
    Vec3 b = v3(4.0, 5.0, 6.0);
    Vec3 result = v3_add(a, b);
    ASSERT_FLOAT_EQ(result.x, 5.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 7.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 9.0, 1e-9);
}

TEST(math3d_v3_sub) {
    Vec3 a = v3(4.0, 5.0, 6.0);
    Vec3 b = v3(1.0, 2.0, 3.0);
    Vec3 result = v3_sub(a, b);
    ASSERT_FLOAT_EQ(result.x, 3.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 3.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 3.0, 1e-9);
}

TEST(math3d_v3_scale) {
    Vec3 v = v3(1.0, 2.0, 3.0);
    Vec3 result = v3_scale(v, 2.5);
    ASSERT_FLOAT_EQ(result.x, 2.5, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 5.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 7.5, 1e-9);
}

TEST(math3d_v3_dot) {
    Vec3 a = v3(1.0, 2.0, 3.0);
    Vec3 b = v3(4.0, 5.0, 6.0);
    double result = v3_dot(a, b);
    ASSERT_FLOAT_EQ(result, 32.0, 1e-9); /* 1*4 + 2*5 + 3*6 = 32 */
}

TEST(math3d_v3_cross) {
    Vec3 a = v3(1.0, 0.0, 0.0);
    Vec3 b = v3(0.0, 1.0, 0.0);
    Vec3 result = v3_cross(a, b);
    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 1.0, 1e-9); /* i × j = k */
}

TEST(math3d_v3_len) {
    Vec3 v = v3(3.0, 4.0, 0.0);
    double result = v3_len(v);
    ASSERT_FLOAT_EQ(result, 5.0, 1e-9); /* sqrt(3² + 4²) = 5 */
}

/* Test Quaternion operations */
TEST(math3d_quat_identity) {
    Quat q = quat_identity();
    ASSERT_FLOAT_EQ(q.w, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(q.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.z, 0.0, 1e-9);
}

TEST(math3d_quat_mul) {
    /* Identity × vector should give the same vector */
    Quat identity = quat_identity();
    Quat v = {0, 1, 0, 0}; /* Pure vector quaternion */
    Quat result = quat_mul(identity, v);
    ASSERT_FLOAT_EQ(result.w, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.x, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_quat_axis_angle) {
    /* 90 degree rotation around Z axis */
    Vec3 axis = v3(0, 0, 1);
    Quat q = quat_axis_angle(axis, M_PI / 2.0);

    /* Expected: w = cos(π/4) ≈ 0.7071, z component of axis × (π/4) */
    double expected_w = cos(M_PI / 4.0);
    ASSERT_FLOAT_EQ(q.w, expected_w, 1e-6);
    ASSERT_FLOAT_EQ(q.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.z, expected_w, 1e-6);
}

TEST(math3d_quat_normalize) {
    Quat q = {2.0, 0.0, 0.0, 0.0};
    Quat result = quat_normalize(q);
    ASSERT_FLOAT_EQ(result.w, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_quat_normalize_zero) {
    Quat q = {0.0, 0.0, 0.0, 0.0};
    Quat result = quat_normalize(q);
    ASSERT_FLOAT_EQ(result.w, 1.0, 1e-9); /* Should return identity for zero */
}

TEST(math3d_quat_rotate) {
    /* Rotate (1, 0, 0) by 90 degrees around Z → (0, 1, 0) */
    Quat q = quat_axis_angle(v3(0, 0, 1), M_PI / 2.0);
    Vec3 v = v3(1, 0, 0);
    Vec3 result = quat_rotate(q, v);

    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-6);
    ASSERT_FLOAT_EQ(result.y, 1.0, 1e-6);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_quat_yaw) {
    /* Identity quaternion → yaw = 0 */
    Quat q = quat_identity();
    ASSERT_FLOAT_EQ(quat_yaw(q), 0.0, 1e-9);
}

TEST(math3d_quat_yaw_90deg) {
    /* 90 degree rotation around Z → yaw = π/2 */
    Quat q = quat_axis_angle(v3(0, 0, 1), M_PI / 2.0);
    ASSERT_FLOAT_EQ(quat_yaw(q), M_PI / 2.0, 1e-6);
}

TEST(math3d_clamp) {
    ASSERT_FLOAT_EQ(clampd(0.5, 0.0, 1.0), 0.5, 1e-9);
    ASSERT_FLOAT_EQ(clampd(-1.0, 0.0, 1.0), 0.0, 1e-9);
    ASSERT_FLOAT_EQ(clampd(2.0, 0.0, 1.0), 1.0, 1e-9);
}

/* Test function for main.c */
int math3d_tests(void) {
    printf("Testing math3d.h functions\n\n");

    RUN_TEST(math3d_v3_creation);
    RUN_TEST(math3d_v3_add);
    RUN_TEST(math3d_v3_sub);
    RUN_TEST(math3d_v3_scale);
    RUN_TEST(math3d_v3_dot);
    RUN_TEST(math3d_v3_cross);
    RUN_TEST(math3d_v3_len);

    RUN_TEST(math3d_quat_identity);
    RUN_TEST(math3d_quat_mul);
    RUN_TEST(math3d_quat_axis_angle);
    RUN_TEST(math3d_quat_normalize);
    RUN_TEST(math3d_quat_normalize_zero);
    RUN_TEST(math3d_quat_rotate);
    RUN_TEST(math3d_quat_yaw);
    RUN_TEST(math3d_quat_yaw_90deg);

    RUN_TEST(math3d_clamp);

    return 0;
}
