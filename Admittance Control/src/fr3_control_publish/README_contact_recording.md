# 接触点记录和三维重建系统

这个系统用于记录机器人末端执行器在扫描过程中的接触点，并生成点云数据用于三维重建。

## 功能特性

- **自动接触检测**：基于力阈值自动检测接触事件
- **多源数据记录**：支持来自控制器和力传感器的接触点记录
- **实时可视化**：提供接触点的实时可视化标记
- **数据导出**：支持CSV和PLY格式的数据导出
- **数据分析**：提供Python脚本进行数据分析和可视化

## 系统组件

### 1. 接触点记录器 (contact_point_recorder)
- 订阅末端执行器位姿和力传感器数据
- 自动检测接触事件
- 记录接触点的位置、法向量和力信息
- 发布可视化标记

### 2. 混合力控制控制器 (hybridforcetangentialcontroller)
- 执行扫描任务
- 在测力完成时发布接触点信息
- 支持多种扫描状态

### 3. 可视化脚本 (visualize_contact_points.py)
- 加载和分析接触点数据
- 生成3D可视化图像
- 提供数据统计信息

## 安装和编译

```bash
# 编译项目
cd /home/mscrobotics2425laptop16/mujoco_ros_ws
colcon build --packages-select fr3_control_publish

# 设置环境
source install/setup.bash
```

## 使用方法

### 1. 启动接触点记录系统

```bash
# 方法1：使用启动文件
ros2 launch fr3_control_publish contact_scanning.launch.py

# 方法2：手动启动节点
ros2 run fr3_control_publish contact_point_recorder
```

### 2. 启动控制器（在另一个终端）

```bash
ros2 run fr3_control_publish hybridforcetangentialcontroller
```

### 3. 启动其他必要节点

```bash
# 启动末端位姿桥接器
ros2 run fr3_control_publish ee_pose_bridge

# 启动MuJoCo仿真（如果需要）
ros2 run fr3_control_publish mujoco_sim_node
```

### 4. 可视化接触点

```bash
# 实时可视化（使用RViz）
rviz2

# 在RViz中添加MarkerArray显示，话题为 /contact_point_markers
```

### 5. 分析记录的数据

```bash
# 可视化接触点数据
python3 src/fr3_control_publish/scripts/visualize_contact_points.py ./contact_data/contact_points.csv

# 保存图像
python3 src/fr3_control_publish/scripts/visualize_contact_points.py ./contact_data/contact_points.csv --output contact_visualization.png --surface surface_plot.png
```

## 参数配置

### 接触点记录器参数

- `output_directory`: 数据输出目录 (默认: "./contact_data")
- `force_threshold`: 接触力阈值 (默认: 5.0 N)
- `min_contact_duration`: 最小接触持续时间 (默认: 0.1 s)
- `enable_visualization`: 是否启用可视化 (默认: true)
- `ee_pose_topic`: 末端位姿话题 (默认: "/ee_pose")
- `wrench_topic`: 力传感器话题 (默认: "/touch_tip/wrench")
- `contact_points_topic`: 接触点话题 (默认: "/contact_points")

### 控制器参数

- `f_high`: 接触阈值 (默认: 25.0 N)
- `f_low`: 离开阈值 (默认: 0.5 N)
- `step_length`: 主轴离散步长 (默认: 0.01 m)
- `lateral_step`: 横向离散步长 (默认: 0.01 m)

## 输出文件

### 数据文件
- `contact_points.csv`: 包含所有接触点信息的CSV文件
- `contact_points.ply`: PLY格式的点云文件

### CSV文件格式
```csv
id,timestamp,x,y,z,nx,ny,nz,fx,fy,fz,frame_id,state_name
1,1234567890.123,0.532,0.754,0.123,0.0,0.0,1.0,10.5,2.3,15.2,world,ALIGN_ORIENT
```

### PLY文件格式
```
ply
format ascii 1.0
element vertex 100
property float x
property float y
property float z
property float nx
property float ny
property float nz
property float fx
property float fy
property float fz
end_header
0.532 0.754 0.123 0.0 0.0 1.0 10.5 2.3 15.2
...
```

## 可视化功能

### 实时可视化
- 在RViz中显示接触点标记
- 根据力大小使用不同颜色
- 显示法向量箭头

### 离线分析
- 3D散点图显示接触点分布
- 根据力大小着色
- 2D投影图显示扫描轨迹
- 数据统计分析

## 故障排除

### 常见问题

1. **没有记录到接触点**
   - 检查力传感器数据是否正常
   - 调整 `force_threshold` 参数
   - 确认话题名称是否正确

2. **可视化不显示**
   - 检查RViz中的MarkerArray话题设置
   - 确认 `enable_visualization` 参数为true

3. **数据文件为空**
   - 检查输出目录权限
   - 确认扫描任务正在运行
   - 检查话题连接状态

### 调试命令

```bash
# 检查话题
ros2 topic list
ros2 topic echo /contact_points
ros2 topic echo /touch_tip/wrench

# 检查节点状态
ros2 node list
ros2 node info /contact_point_recorder

# 查看参数
ros2 param list /contact_point_recorder
```

## 扩展功能

### 自定义可视化
可以修改 `visualize_contact_points.py` 脚本来添加自定义的可视化功能。

### 数据后处理
可以使用其他工具（如MeshLab、CloudCompare）来进一步处理PLY文件。

### 三维重建
虽然当前系统不包含自动三维重建功能，但可以使用其他工具基于PLY文件进行表面重建。

## 贡献

欢迎提交问题和改进建议！ 