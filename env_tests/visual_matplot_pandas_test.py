
import pandas as pd
import matplotlib.pyplot as plt

# 模拟数据
data = {'episode_return': [10, 20, 15, 30, 25]}
df = pd.DataFrame(data)

plt.figure(figsize=(10, 4))
plt.title('Training episodes returns')
plt.xlabel('Training episodes')
plt.ylabel('Episode return')
plt.plot(df['episode_return'])
plt.show()
