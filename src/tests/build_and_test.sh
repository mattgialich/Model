#!/bin/bash

cd /Users/mattastroforge/Desktop/Model/src/tests

echo "=== Cleaning ==="
make clean 2>/dev/null || true

echo ""
echo "=== Building tests ==="
if make tests; then
    echo ""
    echo "=== Running tests ==="
    ./tests
else
    echo "Build failed!"
fi
