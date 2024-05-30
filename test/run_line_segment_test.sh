#!/usr/bin/env bash

# This script runs the line segment fitting test.  

set -e

# Get the path to the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"
# apollo-master/ultrasonic_detection$
cd "$PROJECT_ROOT"

# Build
bazel build test:fit_line_segment_test

if [ $? -eq 0 ]; then
  echo "Building parking_spot GUI succeeded."
else
  echo "Building parking_spot GUI failed." >&2
fi

# Run
source ../cyber/setup.bash

.././bazel-bin/ultrasonic_detection/test/fit_line_segment_test