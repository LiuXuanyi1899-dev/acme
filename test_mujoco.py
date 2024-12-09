
#检查环境变量
import os
import subprocess

# Get LD_LIBRARY_PATH from os.environ
python_ld_library_path = os.environ.get("LD_LIBRARY_PATH")

# Get LD_LIBRARY_PATH from bash
bash_ld_library_path = subprocess.check_output("echo $LD_LIBRARY_PATH", shell=True, text=True).strip()

# Print both paths
print("LD_LIBRARY_PATH from Python os.environ:", python_ld_library_path)
print("LD_LIBRARY_PATH from Bash:", bash_ld_library_path)

# Compare and raise an error if they differ
if python_ld_library_path != bash_ld_library_path:
    raise ValueError("LD_LIBRARY_PATH mismatch between Python and Bash!")

import matplotlib.animation as animation
print(animation.writers.list())

#检查tensorflow是否能找到gpu设备
import tensorflow as tf
print(tf.config.experimental.list_physical_devices('GPU'))
#检查jax
import jax
import jaxlib


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

#顺便测试mujoco-py
import mujoco_py
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)