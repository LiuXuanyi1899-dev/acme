#能成功import完所有的lib就算成功
from typing import Optional
import collections
from dm_control import suite as dm_suite
import dm_env
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from acme import specs
from acme import wrappers
from acme.agents.jax import d4pg
from acme.jax import experiments
from acme.utils import loggers

import matplotlib.animation as animation
print(animation.writers.list()) #预期输出 ['pillow', 'ffmpeg', 'ffmpeg_file', 'html']
#顺便测试mujoco-py
import os
import mujoco_py
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)
