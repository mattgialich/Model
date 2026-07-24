# Model — a robotics world model in C

A fast world model for robotics: a flat ground plane, a fixed-timestep
simulation loop, and swappable robot models defined as **actuator trees** —
each robot is a tree of links with joints, dimensions, actuator limits, and
(dynamics-ready) mass/inertia properties.

The simulation core is headless and has zero dependencies beyond libc/libm —
it runs at millions of steps per second. A raylib 3D viewer is an optional
add-on that watches the sim; the sim never depends on graphics.

## Build & run

```sh
make                    # headless simulator -> ./model
make test               # build + run the unit test suite (src/tests)
make viewer             # 3D viewer -> ./model_view  (needs: brew install raylib)

./model car 10          # run the car headless for 10 sim-seconds
./model quadruped 30
./model biped

./model_view            # 3D window; keys 1/2/3 switch robots, R resets,
                        # mouse drag orbits, wheel zooms
```

## Layout

```
include/
  math3d.h    vectors + quaternions (Z-up, doubles, all inline)
  robot.h     Link / Robot — the actuator tree and its design properties
  control.h   Controller + Observation — the intelligence-model interface
  world.h     World — ground plane, gravity, timestep, robot, controller
  robots.h    RobotModel registry
src/
  robot.c     kinematic stepping, forward kinematics, base propagation
  world.c     the observe -> control -> step loop
  main.c      headless entry point
  viewer.c    optional raylib 3D viewer
robots/
  registry.c  table of available robots
  car.c       4-wheel car, Ackermann steering, bicycle-model base motion
  quadruped.c 4 legs x (hip, knee), trot gait demo
  biped.c     2 legs x (hip, knee, ankle), walk cycle demo
```

## How a robot is defined

A robot is an array of `Link` structs forming a tree (each link names its
parent). Every link carries its full design description:

- **joint**: revolute / prismatic / wheel (continuous) / fixed, plus its
  axis and mounting transform (`origin_pos`, `origin_rot`)
- **actuator**: mode (position- or velocity-controlled), position limits,
  max velocity, max torque, gear ratio
- **geometry**: box / cylinder / sphere with dimensions — this is where
  lengths live (e.g. the quadruped's thigh is a box of length `UPPER_LEN`
  whose child joint mounts at `(0, 0, -UPPER_LEN)`)
- **inertial**: mass, center of mass, inertia — unused by the kinematic
  stepper today, carried so full dynamics can be added without redesign

Each robot file (`robots/*.c`) is just a `build()` function that assembles
this tree from named design constants at the top of the file, plus a
`base_update` callback describing how the floating base moves (the car
integrates a kinematic bicycle model from its wheel/steer state; the legged
robots integrate a commanded body twist at stand height — to be replaced by
contact dynamics later).

### Adding a new robot

1. Copy `robots/quadruped.c` to `robots/<name>.c`; edit the design constants
   and the tree in `build()`.
2. Declare it and add one row in `robots/registry.c`.
3. Add the file to `CORE_SRC` in the `Makefile`.

## The intelligence-model interface

`include/control.h` is the seam. Every tick:

```
Observation (time, q[], dq[], base pose)  ->  Controller.step()
    ->  cmd[] (one value per actuator)  +  base_cmd (vx, vy, yaw rate)
```

Commands are interpreted per-actuator: a target position for position-mode
joints (chased at the actuator's `max_vel`), a target velocity for wheels.
To hook up a learned policy, implement a `Controller` whose `step` marshals
the observation into your model and copies its output into `cmd[]` — nothing
else changes. The demo controllers in `robots/*.c` (sine steering, trot
gait, walk cycle) show the pattern.

## Roadmap ideas

- Full rigid-body dynamics (the mass/inertia fields are already in place):
  gravity, torque-limited joints, ground contact for the legged robots
- Sensors in the Observation (IMU, foot contact, joint torque)
- A socket/IPC bridge so an external intelligence model can drive the
  Controller interface from another process
# Model
# Model
