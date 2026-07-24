#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to tests directory
cd "$SCRIPT_DIR"

echo "Building tests..."
make clean 2>/dev/null || true

# Build
if make tests; then
    echo ""
    echo "Running tests..."
    ./tests
else
    echo "Build failed!"
    exit 1
fi