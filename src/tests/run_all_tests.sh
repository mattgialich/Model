#!/bin/bash
#
# Build and run the test suite with a direct compiler invocation (no make).
# The source list mirrors CORE_SRC / TEST_SRCS in this directory's Makefile —
# keep the two in sync when adding a module or a test file.
set -e

# Get the directory where this script is located, following symlinks so the
# script still works when invoked through a link on $PATH.
SOURCE="${BASH_SOURCE[0]}"
while [ -L "$SOURCE" ]; do
    LINK_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$( readlink "$SOURCE" )"
    [[ "$SOURCE" != /* ]] && SOURCE="$LINK_DIR/$SOURCE"
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
ROOT="$( cd -P "$SCRIPT_DIR/../.." && pwd )"
CC="${CC:-cc}"

echo "Building tests..."
"$CC" -O2 -std=c11 -Wall -Wextra -I"$ROOT/include" \
    "$SCRIPT_DIR/test_math3d.c" \
    "$SCRIPT_DIR/test_robot.c" \
    "$SCRIPT_DIR/test_world.c" \
    "$SCRIPT_DIR/test_registry.c" \
    "$SCRIPT_DIR/test_car.c" \
    "$SCRIPT_DIR/test_quadruped.c" \
    "$SCRIPT_DIR/test_biped.c" \
    "$SCRIPT_DIR/test_biped_balance.c" \
    "$SCRIPT_DIR/main.c" \
    "$ROOT/src/robot.c" \
    "$ROOT/src/world.c" \
    "$ROOT/robots/registry.c" \
    "$ROOT/robots/car.c" \
    "$ROOT/robots/quadruped.c" \
    "$ROOT/robots/biped.c" \
    "$ROOT/robots/biped_balance.c" \
    -o "$SCRIPT_DIR/tests" -lm

echo ""
echo "Running tests..."
"$SCRIPT_DIR/tests"
