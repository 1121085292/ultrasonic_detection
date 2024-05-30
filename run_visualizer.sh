#!/usr/bin/env bash

# This script runs the visualizer for the parking spot detection.  

set -e

# Get the path to the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"

cd "$PROJECT_ROOT"

# Build
bazel build ultrasonic_detection/visualizer:parking_spot

if [ $? -eq 0 ]; then
  echo "Building parking_spot GUI succeeded."
else
  echo "Building parking_spot GUI failed." >&2
fi

# Run
source cyber/setup.bash

./bazel-bin/ultrasonic_detection/visualizer/parking_spot