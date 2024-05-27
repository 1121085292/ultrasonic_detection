#!/usr/bin/env bash

# This script runs the ultrasonic perception component.  

set -e

# Get the path to the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"

cd "$PROJECT_ROOT"

# Build the ultrasonic perception component
bazel build ultrasonic_detection/test:fit_line_segment_test

if [ $? -eq 0 ]; then
  echo "Building parking_spot GUI succeeded."
else
  echo "Building parking_spot GUI failed." >&2
fi

# Run the ultrasonic perception component
source cyber/setup.bash

./bazel-bin/ultrasonic_detection/test/fit_line_segment_test