# Unit tests Makefile

CC      = cc
LDLIBS  = -lm

# Find the directory where this Makefile lives so it works from any cwd
THIS_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
ROOT     := $(THIS_DIR)/../..

CFLAGS  = -O2 -std=c11 -Wall -Wextra -I$(ROOT)/include

CORE_SRC = $(ROOT)/src/robot.c \
           $(ROOT)/src/world.c \
           $(ROOT)/robots/registry.c \
           $(ROOT)/robots/car.c \
           $(ROOT)/robots/quadruped.c \
           $(ROOT)/robots/biped.c \
           $(ROOT)/robots/biped_balance.c

TEST_SRCS = $(THIS_DIR)/test_math3d.c \
            $(THIS_DIR)/test_robot.c \
            $(THIS_DIR)/test_world.c \
            $(THIS_DIR)/test_registry.c \
            $(THIS_DIR)/test_car.c \
            $(THIS_DIR)/test_quadruped.c \
            $(THIS_DIR)/test_biped.c \
            $(THIS_DIR)/test_biped_balance.c

MAIN_SRC = $(THIS_DIR)/main.c

tests: $(CORE_SRC) $(TEST_SRCS) $(MAIN_SRC) $(THIS_DIR)/test_runner.h
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $(THIS_DIR)/tests $(LDLIBS)

test: tests
	$(THIS_DIR)/tests

clean:
	rm -f $(THIS_DIR)/tests

.PHONY: test clean
