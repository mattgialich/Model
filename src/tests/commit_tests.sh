#!/bin/bash
#
# Stage and commit everything under src/tests/.
# Uses the repository's own git identity — it does not configure one.
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

# Resolve the repository root from the script's location, not the cwd.
if ! ROOT="$( git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null )"; then
    echo "error: $SCRIPT_DIR is not inside a git repository." >&2
    exit 1
fi

cd "$ROOT"

echo "Staging test files..."
git add src/tests/

if git diff --cached --quiet -- src/tests/; then
    echo "No changes staged under src/tests/ — nothing to commit."
    exit 0
fi

echo ""
echo "Staged changes:"
git diff --cached --stat -- src/tests/

echo ""
echo "Committing..."
git commit -m "Update unit tests"

echo ""
echo "Git status:"
git status --short
