# Unit Testing Framework

This directory contains the unit test suite for the robot simulation code.

## Structure

- `test_runner.h` - Testing framework: assertion macros, per-test timing,
  summary reporter. Shared state is defined once by `main.c` (which sets
  `TEST_RUNNER_MAIN` before including the header).
- `main.c` - Entry point that runs every suite
- `test_math3d.c` - math3d.h (Vec3 / Quat operations)
- `test_robot.c` - robot.c (links, stepping, limits, FK, base twist)
- `test_world.c` - world.c (init/reset/step, observation, command flow)
- `test_registry.c` - robots/registry.c (model lookup, world_load_model)
- `test_car.c` - robots/car.c (build, bicycle-model driving, clamping)
- `test_quadruped.c` - robots/quadruped.c (build, trot demo, limits)
- `test_biped.c` - robots/biped.c (build, walk demo, limits)

## Usage

From the project root:

```bash
make test
```

Or from this directory:

```bash
make tests     # build ./tests
./tests        # run
```

## Timing

Timing uses `CLOCK_MONOTONIC` kept in integer nanoseconds. The `[ RUN ]`
line is printed and stdout flushed *before* the timer starts, so console
I/O is never counted in a test's duration. The summary reports:

- per-test duration
- a per-module breakdown grouped by test-name prefix (every test is
  counted — nothing is silently dropped)
- the sum of test bodies vs. total harness wall time (which includes
  printing and bookkeeping), so the two are never conflated

```
Timing breakdown (test bodies only):
  math3d        16 tests      0.001 ms
  robot         18 tests      0.179 ms
  ...
  total         61 tests      2.598 ms

Total wall time (incl. output):     2.679 ms
```

## Writing New Tests

1. Create `test_<module>.c` in this directory
2. Include the module header and `test_runner.h`
3. Define tests with the `TEST(name)` macro — name them `<module>_...`
   so the timing breakdown groups them
4. Assert with:
   - `ASSERT_TRUE(cond)` / `ASSERT_FALSE(cond)`
   - `ASSERT_EQ(a, b)` / `ASSERT_NE(a, b)` for integers
   - `ASSERT_FLOAT_EQ(a, b, eps)` for floating point
   - `ASSERT_STRING_EQ(a, b)` / `ASSERT_STRING_NE(a, b)`
5. Add a `<module>_tests()` function that `RUN_TEST`s each test
6. Register it in `main.c` and add the file to `TEST_SRCS` in the Makefile

## Example Test

```c
#include "math3d.h"
#include "test_runner.h"

TEST(math3d_v3_add) {
    Vec3 result = v3_add(v3(1, 2, 3), v3(4, 5, 6));
    ASSERT_FLOAT_EQ(result.x, 5.0, 1e-9);
    ASSERT_FLOAT_EQ(result.y, 7.0, 1e-9);
    ASSERT_FLOAT_EQ(result.z, 9.0, 1e-9);
}
```
