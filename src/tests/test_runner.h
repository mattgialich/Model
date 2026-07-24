/* test_runner.h — Lightweight unit testing framework for the model.
 *
 * Usage:
 *   1. Include this header in your test file
 *   2. Define TEST(name) { ... } and register with RUN_TEST(name)
 *   3. Call run_all_tests() at the end of main
 *
 * Features:
 *   - Simple assertion macros (ASSERT_EQ, ASSERT_FLOAT_EQ, etc.)
 *   - Floating-point comparisons with epsilon
 *   - String comparison
 *   - Summary report with per-test and per-module timing
 *
 * Timing accuracy:
 *   - CLOCK_MONOTONIC via clock_gettime, kept in integer nanoseconds
 *     (no floating-point accumulation error)
 *   - stdout is flushed and the RUN line printed BEFORE the timer starts,
 *     so console I/O is never counted in a test's duration
 *   - The summary separates "sum of test bodies" from total harness
 *     wall time (which includes printing and bookkeeping)
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

/* High-resolution monotonic clock, integer nanoseconds */
static inline uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Test result tracking.
 * The state is shared by every test file, so it is declared extern here and
 * defined exactly once: the test suite's main.c sets TEST_RUNNER_MAIN before
 * including this header. */
typedef struct {
    const char *name;
    int passed;
    uint64_t duration_ns;
} TestCase;

extern TestCase *test_cases;
extern int test_count;
extern int test_capacity;
extern int tests_passed;
extern uint64_t harness_start_ns;

#ifdef TEST_RUNNER_MAIN
TestCase *test_cases = NULL;
int test_count = 0;
int test_capacity = 0;
int tests_passed = 0;
uint64_t harness_start_ns = 0;
#endif

/* Macros for defining tests */
#define TEST(name) \
    static void test_##name(void)

#define RUN_TEST(test_name) \
    do { \
        if (test_count >= test_capacity) { \
            test_capacity = test_capacity ? test_capacity * 2 : 16; \
            test_cases = realloc(test_cases, test_capacity * sizeof(TestCase)); \
        } \
        test_cases[test_count].name = #test_name; \
        test_cases[test_count].passed = 1; \
        test_cases[test_count].duration_ns = 0; \
        test_count++; \
        \
        printf("[ RUN      ] %-32s ", #test_name); \
        fflush(stdout); \
        uint64_t before__ = now_ns(); \
        test_##test_name(); \
        uint64_t after__ = now_ns(); \
        test_cases[test_count - 1].duration_ns = after__ - before__; \
        \
        if (test_cases[test_count - 1].passed) { \
            tests_passed++; \
            printf("[       OK ] %9.3f ms\n", \
                   test_cases[test_count - 1].duration_ns / 1e6); \
        } else { \
            printf("[  FAILED  ] %9.3f ms\n", \
                   test_cases[test_count - 1].duration_ns / 1e6); \
        } \
    } while (0)

/* Assertion macros */
#define TEST_FAIL_() (test_cases[test_count - 1].passed = 0)

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            printf("\n  Error: ASSERT_TRUE(%s) failed at %s:%d\n", \
                   #cond, __FILE__, __LINE__); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_FALSE(cond) \
    do { \
        if (cond) { \
            printf("\n  Error: ASSERT_FALSE(%s) failed at %s:%d\n", \
                   #cond, __FILE__, __LINE__); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_EQ(a, b) \
    do { \
        if ((a) != (b)) { \
            printf("\n  Error: ASSERT_EQ(%s, %s) failed at %s:%d\n", \
                   #a, #b, __FILE__, __LINE__); \
            printf("    Expected: %ld, Got: %ld\n", (long)(b), (long)(a)); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_NE(a, b) \
    do { \
        if ((a) == (b)) { \
            printf("\n  Error: ASSERT_NE(%s, %s) failed at %s:%d\n", \
                   #a, #b, __FILE__, __LINE__); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_FLOAT_EQ(a, b, eps) \
    do { \
        double a__ = (a), b__ = (b); \
        if (!(fabs(a__ - b__) <= (eps))) { \
            printf("\n  Error: ASSERT_FLOAT_EQ(%s, %s) failed at %s:%d\n", \
                   #a, #b, __FILE__, __LINE__); \
            printf("    Expected: %.9f, Got: %.9f (diff: %.3g)\n", \
                   b__, a__, fabs(a__ - b__)); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_STRING_EQ(a, b) \
    do { \
        if (strcmp((a), (b)) != 0) { \
            printf("\n  Error: ASSERT_STRING_EQ(%s, %s) failed at %s:%d\n", \
                   #a, #b, __FILE__, __LINE__); \
            printf("    Expected: \"%s\", Got: \"%s\"\n", (b), (a)); \
            TEST_FAIL_(); \
        } \
    } while (0)

#define ASSERT_STRING_NE(a, b) \
    do { \
        if (strcmp((a), (b)) == 0) { \
            printf("\n  Error: ASSERT_STRING_NE(%s, %s) failed at %s:%d\n", \
                   #a, #b, __FILE__, __LINE__); \
            printf("    Strings are equal: \"%s\"\n", (a)); \
            TEST_FAIL_(); \
        } \
    } while (0)

/* The summary reporter is only needed in the suite's entry point */
#ifdef TEST_RUNNER_MAIN

/* Module of a test = its name up to the first '_' ("math3d_v3_add" -> "math3d") */
static int module_len_(const char *name) {
    const char *u = strchr(name, '_');
    return u ? (int)(u - name) : (int)strlen(name);
}

/* Run all tests and print summary with timing */
static int run_all_tests(void) {
    uint64_t harness_ns = now_ns() - harness_start_ns;

    printf("\n========================================\n");
    printf("Test Summary\n");
    printf("========================================\n");
    printf("Tests run:     %d\n", test_count);
    printf("Passed:        %d\n", tests_passed);
    printf("Failed:        %d\n", test_count - tests_passed);

    if (test_count - tests_passed > 0) {
        printf("\nFailed tests:\n");
        for (int i = 0; i < test_count; i++)
            if (!test_cases[i].passed)
                printf("  %s\n", test_cases[i].name);
    }

    /* Per-module timing, grouped dynamically by name prefix so no test is
     * ever silently dropped from the breakdown. */
    printf("\nTiming breakdown (test bodies only):\n");
    uint64_t total_body_ns = 0;
    for (int i = 0; i < test_count; i++) {
        total_body_ns += test_cases[i].duration_ns;

        /* print each module once, at its first test */
        int len = module_len_(test_cases[i].name);
        int seen = 0;
        for (int j = 0; j < i && !seen; j++)
            seen = module_len_(test_cases[j].name) == len &&
                   strncmp(test_cases[j].name, test_cases[i].name, len) == 0;
        if (seen) continue;

        uint64_t module_ns = 0;
        int module_tests = 0;
        for (int j = 0; j < test_count; j++) {
            if (module_len_(test_cases[j].name) == len &&
                strncmp(test_cases[j].name, test_cases[i].name, len) == 0) {
                module_ns += test_cases[j].duration_ns;
                module_tests++;
            }
        }
        printf("  %-12.*s %3d tests  %9.3f ms\n",
               len, test_cases[i].name, module_tests, module_ns / 1e6);
    }
    printf("  %-12s %3d tests  %9.3f ms\n", "total", test_count,
           total_body_ns / 1e6);

    printf("\nTotal wall time (incl. output): %9.3f ms\n", harness_ns / 1e6);
    printf("========================================\n");

    free(test_cases);
    test_cases = NULL;

    return (tests_passed == test_count) ? 0 : 1;
}

/* Start the harness wall clock */
static void init_timing(void) {
    harness_start_ns = now_ns();
}

#endif /* TEST_RUNNER_MAIN */

#endif /* TEST_RUNNER_H */
