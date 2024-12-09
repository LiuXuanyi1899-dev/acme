import tensorflow as tf
from acme.agents.jax import d4pg
import helpers
#/home/xuanyi/acme/20241209-130649/checkpoints/learner
def load_model_from_checkpoint(checkpoint_path, environment_spec):
    """从检查点加载模型."""
    # 创建与训练时一致的网络
    networks = d4pg.make_networks(
        environment_spec,
        policy_layer_sizes=(256, 256, 256),
        critic_layer_sizes=(256, 256, 256),
        vmin=-1000.0,  # 根据训练时的 vmax
        vmax=1000.0,
    )

    # 初始化检查点
    checkpoint = tf.train.Checkpoint(networks=networks)

    # 恢复检查点
    status = checkpoint.restore(checkpoint_path)
    status.assert_existing_objects_matched()  # 检查所有对象匹配

    print("模型加载完成.")
    return networks.policy

spec=helpers.make_environment("gym", "HalfCheetah-v2")
load_model_from_checkpoint("/home/xuanyi/acme/20241209-130649/checkpoints/learner",spec)