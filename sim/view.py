"""Open the model in the MuJoCo viewer. Usage: uv run python view.py [start angle in degrees from upright]"""

import math
import sys

import mujoco
import mujoco.viewer

import model

m = model.build_single(model.load_params())
d = mujoco.MjData(m)
d.qpos[1] = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 5.0)
mujoco.viewer.launch(m, d)
