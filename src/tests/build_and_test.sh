#!/bin/bash
# Build and run the unit test suite.
set -e
cd "$(dirname "$0")"
make tests
./tests
