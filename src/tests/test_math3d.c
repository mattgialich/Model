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

TEST(math3d_v3_len_arbitrary) {
    /* Test v3_len with non-trivial values */
    Vec3 v = v3(1.0, 2.0, 2.0);
    double result = v3_len(v);
    ASSERT_FLOAT_EQ(result, 3.0, 1e-9); /* sqrt(1 + 4 + 4) = 3 */
}

TEST(math3d_v3_len_zero) {
    /* Test v3_len with zero vector */
    Vec3 v = v3(0.0, 0.0, 0.0);
    double result = v3_len(v);
    ASSERT_FLOAT_EQ(result, 0.0, 1e-9);
}

TEST(math3d_v3_len_negative) {
    /* Test v3_len with negative components (squared anyway) */
    Vec3 v = v3(-3.0, -4.0, 0.0);
    double result = v3_len(v);
    ASSERT_FLOAT_EQ(result, 5.0, 1e-9); /* sqrt(9 + 16) = 5 */
}

TEST(math3d_v3_scale_negative) {
    /* Test scaling with negative factor */
    Vec3 v = v3(1.0, 2.0, 3.0);
    Vec3 result = v3_scale(v, -2.0);
    ASSERT_FLOAT_EQ(result.x, -2.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, -4.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, -6.0, 1e-9);
}

TEST(math3d_v3_dot_negative) {
    /* Test dot product with negative vectors */
    Vec3 a = v3(-1.0, -2.0, -3.0);
    Vec3 b = v3(1.0, 2.0, 3.0);
    double result = v3_dot(a, b);
    ASSERT_FLOAT_EQ(result, -14.0, 1e-9); /* negative dot product */
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

TEST(math3d_quat_axis_angle_180deg) {
    /* 180 degree rotation around Z axis */
    Vec3 axis = v3(0, 0, 1);
    Quat q = quat_axis_angle(axis, M_PI);

    /* Expected: w = cos(π/2) = 0 */
    ASSERT_FLOAT_EQ(q.w, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.z, 1.0, 1e-9); /* sin(π/2) = 1 */
}

TEST(math3d_quat_axis_angle_zero) {
    /* Zero degree rotation (should be identity) */
    Vec3 axis = v3(1, 0, 0);
    Quat q = quat_axis_angle(axis, 0.0);

    ASSERT_FLOAT_EQ(q.w, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(q.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(q.z, 0.0, 1e-9);
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

TEST(math3d_quat_normalize_arbitrary) {
    Quat q = {1.0, 2.0, 3.0, 4.0};
    Quat result = quat_normalize(q);
    
    /* Check that normalized quaternion has length 1 */
    double len = sqrt(result.w*result.w + result.x*result.x + 
                      result.y*result.y + result.z*result.z);
    ASSERT_FLOAT_EQ(len, 1.0, 1e-6);
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

TEST(math3d_quat_rotate_180deg) {
    /* Rotate (1, 0, 0) by 180 degrees around Z → (-1, 0, 0) */
    Quat q = quat_axis_angle(v3(0, 0, 1), M_PI);
    Vec3 v = v3(1, 0, 0);
    Vec3 result = quat_rotate(q, v);

    ASSERT_FLOAT_EQ(result.x, -1.0, 1e-6);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-6);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_quat_rotate_identity) {
    /* Rotate by identity should give same vector */
    Quat q = quat_identity();
    Vec3 v = v3(1, 2, 3);
    Vec3 result = quat_rotate(q, v);

    ASSERT_FLOAT_EQ(result.x, 1.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 2.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 3.0, 1e-9);
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

TEST(math3d_quat_yaw_180deg) {
    /* 180 degree rotation around Z → yaw = π */
    Quat q = quat_axis_angle(v3(0, 0, 1), M_PI);
    ASSERT_FLOAT_EQ(quat_yaw(q), M_PI, 1e-6);
}

TEST(math3d_quat_yaw_minus90deg) {
    /* -90 degree rotation around Z → yaw = -π/2 */
    Quat q = quat_axis_angle(v3(0, 0, 1), -M_PI / 2.0);
    ASSERT_FLOAT_EQ(quat_yaw(q), -M_PI / 2.0, 1e-6);
}

TEST(math3d_clamp) {
    ASSERT_FLOAT_EQ(clampd(0.5, 0.0, 1.0), 0.5, 1e-9);
    ASSERT_FLOAT_EQ(clampd(-1.0, 0.0, 1.0), 0.0, 1e-9);
    ASSERT_FLOAT_EQ(clampd(2.0, 0.0, 1.0), 1.0, 1e-9);
}

TEST(math3d_clamp_edge_low) {
    /* Test clamp at exact lower bound */
    ASSERT_FLOAT_EQ(clampd(0.0, 0.0, 1.0), 0.0, 1e-9);
}

TEST(math3d_clamp_edge_high) {
    /* Test clamp at exact upper bound */
    ASSERT_FLOAT_EQ(clampd(1.0, 0.0, 1.0), 1.0, 1e-9);
}

TEST(math3d_clamp_negative_range) {
    /* Test clamp with negative range */
    ASSERT_FLOAT_EQ(clampd(-5.0, -10.0, -2.0), -5.0, 1e-9);
    ASSERT_FLOAT_EQ(clampd(-20.0, -10.0, -2.0), -10.0, 1e-9);
    ASSERT_FLOAT_EQ(clampd(-1.0, -10.0, -2.0), -2.0, 1e-9);
}

TEST(math3d_cross_parallel) {
    /* Cross product of parallel vectors is zero */
    Vec3 a = v3(1.0, 2.0, 3.0);
    Vec3 b = v3(2.0, 4.0, 6.0);
    Vec3 result = v3_cross(a, b);
    
    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_cross_antiparallel) {
    /* Cross product of antiparallel vectors is zero */
    Vec3 a = v3(1.0, 2.0, 3.0);
    Vec3 b = v3(-1.0, -2.0, -3.0);
    Vec3 result = v3_cross(a, b);

    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
}

TEST(math3d_dot_orthogonal) {
    /* Dot product of orthogonal vectors is zero */
    Vec3 a = v3(1.0, 0.0, 0.0);
    Vec3 b = v3(0.0, 1.0, 0.0);
    double result = v3_dot(a, b);

    ASSERT_FLOAT_EQ(result, 0.0, 1e-9);
}

TEST(math3d_dot_parallel) {
    /* Dot product of parallel vectors equals product of magnitudes */
    Vec3 a = v3(1.0, 2.0, 2.0);
    Vec3 b = v3(2.0, 4.0, 4.0);
    double result = v3_dot(a, b);

    /* a·b = |a||b|cos(0) = |a||b| */
    double mag_a = v3_len(a);  /* = 3 */
    double mag_b = v3_len(b);  /* = 6 */
    ASSERT_FLOAT_EQ(result, mag_a * mag_b, 1e-9);
}

TEST(math3d_v3_sub_reverse) {
    /* Test that a - b = -(b - a) */
    Vec3 a = v3(4.0, 5.0, 6.0);
    Vec3 b = v3(1.0, 2.0, 3.0);

    Vec3 ab = v3_sub(a, b);
    Vec3 ba = v3_sub(b, a);

    ASSERT_FLOAT_EQ(ab.x, -ba.x, 1e-9);
    ASSERT_FLOAT_EQ(ab.y, -ba.y, 1e-9);
    ASSERT_FLOAT_EQ(ab.z, -ba.z, 1e-9);
}

TEST(math3d_v3_additive_inverse) {
    /* Test that a + (-a) = 0 */
    Vec3 v = v3(1.0, 2.0, 3.0);
    Vec3 neg = v3_scale(v, -1.0);
    Vec3 result = v3_add(v, neg);

    ASSERT_FLOAT_EQ(result.x, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 0.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 0.0, 1e-9);
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
    RUN_TEST(math3d_v3_len_arbitrary);
    RUN_TEST(math3d_v3_len_zero);
    RUN_TEST(math3d_v3_len_negative);
    RUN_TEST(math3d_v3_scale_negative);
    RUN_TEST(math3d_v3_dot_negative);

    RUN_TEST(math3d_quat_identity);
    RUN_TEST(math3d_quat_mul);
    RUN_TEST(math3d_quat_axis_angle);
    RUN_TEST(math3d_quat_axis_angle_180deg);
    RUN_TEST(math3d_quat_axis_angle_zero);
    RUN_TEST(math3d_quat_normalize);
    RUN_TEST(math3d_quat_normalize_zero);
    RUN_TEST(math3d_quat_normalize_arbitrary);
    RUN_TEST(math3d_quat_rotate);
    RUN_TEST(math3d_quat_rotate_180deg);
    RUN_TEST(math3d_quat_rotate_identity);
    RUN_TEST(math3d_quat_yaw);
    RUN_TEST(math3d_quat_yaw_90deg);
    RUN_TEST(math3d_quat_yaw_180deg);
    RUN_TEST(math3d_quat_yaw_minus90deg);

    RUN_TEST(math3d_clamp);
    RUN_TEST(math3d_clamp_edge_low);
    RUN_TEST(math3d_clamp_edge_high);
    RUN_TEST(math3d_clamp_negative_range);

    RUN_TEST(math3d_cross_parallel);
    RUN_TEST(math3d_cross_antiparallel);
    RUN_TEST(math3d_dot_orthogonal);
    RUN_TEST(math3d_dot_parallel);
    RUN_TEST(math3d_v3_sub_reverse);
    RUN_TEST(math3d_v3_additive_inverse);

    return 0;
}