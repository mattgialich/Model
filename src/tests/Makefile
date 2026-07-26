# Unit tests Makefile
#
# Every path here is relative to this directory. That is deliberate: make
# splits prerequisite lists on whitespace, so an absolute path containing a
# space cannot be expressed as a prerequisite at all. Staying relative makes
# the build immune to spaces anywhere in the repository's absolute path.
#
# The trade-off is that make's working directory must BE this directory.
# Use `make -C src/tests test` — which is how the root Makefile and the
# scripts here invoke it. The guard below catches any other case.

ifeq ($(wildcard test_runner.h),)
$(error wrong working directory — run: make -C src/tests $(or $(MAKECMDGOALS),test))
endif

CC      = cc
LDLIBS  = -lm
ROOT    = ../..

CFLAGS  = -O2 -std=c11 -Wall -Wextra -I$(ROOT)/include

CORE_SRC = $(ROOT)/src/robot.c \
           $(ROOT)/src/world.c \
           $(ROOT)/robots/registry.c \
           $(ROOT)/robots/car.c \
           $(ROOT)/robots/quadruped.c \
           $(ROOT)/robots/biped.c \
           $(ROOT)/robots/biped_balance.c

TEST_SRCS = test_math3d.c \
            test_robot.c \
            test_world.c \
            test_registry.c \
            test_car.c \
            test_quadruped.c \
            test_biped.c \
            test_biped_balance.c

MAIN_SRC = main.c

tests: $(CORE_SRC) $(TEST_SRCS) $(MAIN_SRC) test_runner.h
	$(CC) $(CFLAGS) $(filter %.c,$^) -o tests $(LDLIBS)

test: tests
	./tests

clean:
	rm -f tests

.PHONY: test clean
