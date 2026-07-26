#!/bin/bash

# Get the directory where this script is located, following symlinks so the
# script still works when invoked through a link on $PATH.
SOURCE="${BASH_SOURCE[0]}"
while [ -L "$SOURCE" ]; do
    LINK_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$( readlink "$SOURCE" )"
    [[ "$SOURCE" != /* ]] && SOURCE="$LINK_DIR/$SOURCE"
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

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
