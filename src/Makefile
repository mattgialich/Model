# Demo Makefile for Model

CC      = cc
CFLAGS  = -O2 -std=c11 -Wall -Wextra -I../include
LDLIBS  = -lm

DEMO_SRC = humanoid_demo.c \
           ../robots/registry.c \
           ../robots/biped.c \
           ../robots/biped_balance.c \
           ../robots/car.c \
           ../robots/quadruped.c \
           robot.c \
           world.c

DEMO = humanoid_demo

all: $(DEMO)

$(DEMO): $(DEMO_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -f $(DEMO)

.PHONY: all clean
