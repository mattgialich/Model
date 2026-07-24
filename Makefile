CC      = cc
CFLAGS  = -O2 -std=c11 -Wall -Wextra -Iinclude
LDLIBS  = -lm

CORE_SRC = src/robot.c src/world.c \
           robots/registry.c robots/car.c robots/quadruped.c robots/biped.c

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib 2>/dev/null)
RAYLIB_LIBS   := $(shell pkg-config --libs raylib 2>/dev/null)

all: model

# headless simulator — no dependencies beyond libc/libm
model: $(CORE_SRC) src/main.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

# 3D viewer — requires raylib (brew install raylib)
viewer: $(CORE_SRC) src/viewer.c
	@if [ -z "$(RAYLIB_LIBS)" ]; then \
	  echo "raylib not found — install with: brew install raylib"; exit 1; fi
	$(CC) $(CFLAGS) $(RAYLIB_CFLAGS) $^ -o model_view $(RAYLIB_LIBS) $(LDLIBS)

# unit tests — build and run the suite in src/tests
test:
	$(MAKE) -C src/tests test

clean:
	rm -f model model_view
	$(MAKE) -C src/tests clean

.PHONY: all viewer test clean
