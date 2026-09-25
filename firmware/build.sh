#!/bin/sh
# Compile the Pico 2 firmware. Usage: firmware/build.sh [--upload -p /dev/cu.usbmodemXXXX]
set -e
cd "$(dirname "$0")"
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 \
  --build-property "compiler.cpp.extra_flags=-I$(pwd) -Wall -Wno-missing-braces" \
  --output-dir build pendulum "$@"
