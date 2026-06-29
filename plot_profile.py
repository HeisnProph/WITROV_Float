import pandas as pd
import matplotlib.pyplot as plt

# ==========================
# 修改成你的CSV文件路径
# ==========================
csv_file = "data.csv"

# 读取CSV
df = pd.read_csv(csv_file)

# 转换时间戳
df["time"] = pd.to_datetime(df["time"], unit="ms")

# 创建三个子图
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# 深度
axs[0].plot(df["time"], df["depth"], linewidth=1.5)
axs[0].set_title("Depth vs Time")
axs[0].set_ylabel("Depth (m)")
axs[0].grid(True)

# 水压
axs[1].plot(df["time"], df["pressure"], linewidth=1.5)
axs[1].set_title("Pressure vs Time")
axs[1].set_ylabel("Pressure")
axs[1].grid(True)

# 水温
axs[2].plot(df["time"], df["temperature"], linewidth=1.5)
axs[2].set_title("Temperature vs Time")
axs[2].set_ylabel("Temperature (°C)")
axs[2].set_xlabel("Time")
axs[2].grid(True)

# 自动调整时间标签
plt.xticks(rotation=30)
plt.tight_layout()

# 显示图像
plt.show()