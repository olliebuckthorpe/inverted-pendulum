#!/bin/sh
# Compile the firmware. Usage: firmware/build.sh [--upload -p /dev/cu.usbmodemXXXX]
# Built for the plain Pico 2 even though the ordered board is a Pico 2 W: same RP2350 and GPIO. The rpipico2w
# target starts the wireless chip at boot (cyw43_arch_init: a PIO state machine, DMA and background IRQs, a risk
# to the encoder PIO and loop jitter); this build never drives its power-enable pin (GP23), so the radio stays
# off. Only the on-board LED (on the wireless chip) is lost, and the firmware doesn't use it.
set -e
cd "$(dirname "$0")"
INC="$(pwd)"
command -v cygpath >/dev/null 2>&1 && INC="$(cygpath -m "$INC")"   # Git Bash on Windows: C:/... not /c/...
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 \
  --build-property "compiler.cpp.extra_flags=-I$INC -Wall -Wno-missing-braces" \
  --output-dir build pendulum "$@"
