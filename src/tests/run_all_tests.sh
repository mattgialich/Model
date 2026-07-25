#!/bin/bash
cd /Users/mattastroforge/Desktop/Model/src/tests

# Compile the test executable
gcc -O2 -std=c11 -Wall -Wextra -I../include \
    test_math3d.c test_robot.c test_world.c main.c \
    ../../src/robot.c ../../src/world.c \
    ../../robots/registry.c robots/car.c robots/quadruped.c robots/biped.c \
    -o tests -lm

# Run the tests
./tests
