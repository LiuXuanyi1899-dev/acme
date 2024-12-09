import jax
print("JAX devices:", jax.devices())
import tensorflow as tf
print("TensorFlow GPU devices:", tf.config.list_physical_devices('GPU'))
print(tf.config.experimental.list_physical_devices('GPU'))
import os
python_ld_library_path = os.environ.get("LD_LIBRARY_PATH")
print("LD_LIBRARY_PATH from Python os.environ:", python_ld_library_path)
