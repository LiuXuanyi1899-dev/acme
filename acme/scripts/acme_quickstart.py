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


def make_environment(seed: int) -> dm_env.Environment:
  environment = dm_suite.load('cartpole', 'balance')

  # Make the observations be a flat vector of all concatenated features.
  environment = wrappers.ConcatObservationWrapper(environment)

  # Wrap the environment so the expected continuous action spec is [-1, 1].
  # Note: this is a no-op on 'control' tasks.
  environment = wrappers.CanonicalSpecWrapper(environment, clip=True)

  # Make sure the environment outputs single-precision floats.
  environment = wrappers.SinglePrecisionWrapper(environment)

  return environment


def network_factory(spec: specs.EnvironmentSpec) -> d4pg.D4PGNetworks:
  return d4pg.make_networks(
      spec,
      # These correspond to sizes of the hidden layers of an MLP.
      policy_layer_sizes=(256, 256),
      critic_layer_sizes=(256, 256),
  )

d4pg_config = d4pg.D4PGConfig(learning_rate=3e-4, sigma=0.2)
d4pg_builder = d4pg.D4PGBuilder(d4pg_config)

# Specify how to log training data: in this case keeping it in memory.
# NOTE: We create a dict to hold the loggers so we can access the data after
# the experiment has run.
logger_dict = collections.defaultdict(loggers.InMemoryLogger)
def logger_factory(
    name: str,
    steps_key: Optional[str] = None,
    task_id: Optional[int] = None,
) -> loggers.Logger:
  del steps_key, task_id
  return logger_dict[name]

experiment_config = experiments.ExperimentConfig(
    builder=d4pg_builder,
    environment_factory=make_environment,
    network_factory=network_factory,
    logger_factory=logger_factory,
    seed=0,
    max_num_actor_steps=50_00)  # Each episode is 1000 steps.

experiments.run_experiment(
    experiment=experiment_config,
    eval_every=100,
    num_eval_episodes=1)


df = pd.DataFrame(logger_dict['evaluator'].data)
plt.figure(figsize=(10, 4))
plt.title('Training episodes returns')
plt.xlabel('Training episodes')
plt.ylabel('Episode return')
plt.plot(df['actor_episodes'], df['episode_return'], label='Training Episodes return')