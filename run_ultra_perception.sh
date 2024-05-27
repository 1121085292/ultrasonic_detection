#!/usr/bin/env bash

# This script runs the ultrasonic perception component.  

set -e

# Get the path to the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"

cd "$PROJECT_ROOT"

# Build the ultrasonic perception component
bazel build ultrasonic_detection:libultrasonic_perception.so

if [ $? -eq 0 ]; then
  echo "Building ultrasonic perception component succeeded."
else
  echo "Building ultrasonic perception component failed." >&2
fi

# Run the ultrasonic perception component
source cyber/setup.bash

cyber_launch start ultrasonic_detection/launch/ultrasonic.launch

if [ $? -eq 0 ]; then    
  echo "Running ultrasonic perception component succeeded."
else
  echo "Running ultrasonic perception component failed." >&2
fi