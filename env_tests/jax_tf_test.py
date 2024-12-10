import os
python_ld_library_path = os.environ.get("LD_LIBRARY_PATH")
print("LD_LIBRARY_PATH from os.environ:", python_ld_library_path)

import jax
print("JAX devices:", jax.devices()) #预期 [cuda(id=0)]
import tensorflow as tf
print("TensorFlow GPU devices:", tf.config.list_physical_devices('GPU')) #预期[PhysicalDevice(name='/physical_device:GPU:0', device_type='GPU')]
