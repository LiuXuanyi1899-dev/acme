# import matplotlib
# import matplotlib.pyplot as plt
#
# # 检查 matplotlib 的当前后端
# print(f"Matplotlib backend: {matplotlib.get_backend()}")
#
# # 尝试绘制一个简单的图形
# try:
#     plt.figure()
#     plt.plot([1, 2, 3], [4, 5, 6], label="Test Line")
#     plt.title("Matplotlib Visualization Test")
#     plt.xlabel("X-axis")
#     plt.ylabel("Y-axis")
#     plt.legend()
#     plt.show()
#     print("Visualization test passed: A window should have popped up.")
# except Exception as e:
#     print(f"Visualization test failed: {e}")
#
#
#
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from IPython.display import HTML
#
# # 生成一些假帧数据（用于模拟视频）
# def generate_test_frames(num_frames=50, frame_size=(100, 100)):
#     frames = []
#     for i in range(num_frames):
#         frame = np.random.rand(*frame_size) * 255  # 随机生成灰度图像
#         frames.append(frame.astype(np.uint8))
#     return frames
#
# # 测试帧数据
# frames = generate_test_frames()
#
# # 检查帧生成是否成功
# print(f"Generated {len(frames)} frames of size {frames[0].shape}.")
#
# # 使用 matplotlib 创建动画
# fig, ax = plt.subplots()
# img = ax.imshow(frames[0], cmap="gray", interpolation="nearest")
# ax.axis("off")
#
# def update(frame):
#     img.set_data(frame)
#     return [img]
#
# # 创建动画
# ani = FuncAnimation(fig, update, frames=frames, interval=50, blit=True)
#
# # 1. 尝试嵌入 HTML 动画
# try:
#     html_animation = ani.to_html5_video()
#     # if 'IPython' in globals():  # 检查是否在 Jupyter Notebook 中
#     #     from IPython.display import HTML
#     #     display(HTML(html_animation))  # Jupyter 环境下嵌入 HTML
#     #     print("HTML animation created and displayed successfully.")
#     # else:
#     print("Running outside of Jupyter Notebook. Skipping HTML display.")
# except Exception as e:
#     print(f"Failed to create HTML animation: {e}")
#
# # 2. 保存动画为 mp4 文件
# try:
#     ani.save("test_animation.mp4", writer="ffmpeg")
#     print("Animation saved as 'test_animation.mp4'.")
# except Exception as e:
#     print(f"Failed to save animation: {e}")
#
# # 3. 显示动画（如果不是 Jupyter Notebook）
# plt.show()
#


from dm_control import suite
from acme import wrappers

# 加载一个 MuJoCo 环境
environment = suite.load(domain_name="cartpole", task_name="balance")

# 包装环境以生成视频
wrapped_env = wrappers.MujocoVideoWrapper(environment, record_every=1)

# 测试视频生成
timestep = wrapped_env.reset()
while not timestep.last():
    action = environment.action_spec().generate_value()
    timestep = wrapped_env.step(action)

# 生成 HTML 文件
html_content = wrapped_env.make_html_animation()
with open("cartpole_video.html", "w") as f:
    f.write(html_content)

print("Video saved as cartpole_video.html")

