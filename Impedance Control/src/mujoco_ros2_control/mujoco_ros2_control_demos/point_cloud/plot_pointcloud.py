import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import glob

# 查找最新的CSV文件
csv_files = glob.glob('./scan_pointcloud*.csv')
if not csv_files:
    print("未找到CSV文件")
    exit()

# 按修改时间排序，获取最新的文件
latest_file = max(csv_files, key=os.path.getmtime)
print(f"使用最新文件: {latest_file}")

# 读取CSV数据
df = pd.read_csv(latest_file)

# 降低绘图密度：每100个点绘制一次
df_sampled = df.iloc[::500].copy()

# 3D散点图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制散点图，不使用颜色条
ax.scatter(df_sampled['x'], df_sampled['y'], df_sampled['z'], 
          c='blue', s=20, alpha=0.7)

# 设置坐标轴标签
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Scan Point Cloud')

# 保持xyz轴比例尺一致
x_range = df_sampled['x'].max() - df_sampled['x'].min()
y_range = df_sampled['y'].max() - df_sampled['y'].min()
z_range = df_sampled['z'].max() - df_sampled['z'].min()
max_range = max(x_range, y_range, z_range)

x_center = (df_sampled['x'].max() + df_sampled['x'].min()) / 2
y_center = (df_sampled['y'].max() + df_sampled['y'].min()) / 2
z_center = (df_sampled['z'].max() + df_sampled['z'].min()) / 2

half_range = max_range / 2
ax.set_xlim(x_center - half_range, x_center + half_range)
ax.set_ylim(y_center - half_range, y_center + half_range)
ax.set_zlim(z_center - half_range, z_center + half_range)

plt.show()
