#!/bin/sh
# Live 3D replay. mjpython can't find uv's libpython on its own, so point it there first.
# Usage: ./watch.sh swingup [--teeth 30] [--length 0.40] [--seed 3]
cd "$(dirname "$0")"
PYLIB=$(uv run python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
DYLD_LIBRARY_PATH="$PYLIB" exec uv run mjpython watch.py "$@"
