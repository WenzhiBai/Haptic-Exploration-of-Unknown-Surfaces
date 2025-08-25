# 接触点记录系统使用说明

## 快速开始

### 1. 编译项目
```bash
cd /home/mscrobotics2425laptop16/mujoco_ros_ws
colcon build --packages-select fr3_control_publish
source install/setup.bash
```

### 2. 启动系统

**终端1：启动接触点记录器**
```bash
ros2 run fr3_control_publish contact_point_recorder
```

**终端2：启动控制器**
```bash
ros2 run fr3_control_publish hybridforcetangentialcontroller
```

**终端3：启动末端位姿桥接器**
```bash
ros2 run fr3_control_publish ee_pose_bridge
```

**终端4：启动MuJoCo仿真（可选）**
```bash
ros2 run fr3_control_publish mujoco_sim_node
```

### 3. 可视化

**实时可视化（RViz）**
```bash
rviz2
```
在RViz中添加MarkerArray显示，话题为 `/contact_point_markers`

**离线数据分析**
```bash
# 等待扫描完成后，分析数据
python3 src/fr3_control_publish/scripts/visualize_contact_points.py ./contact_data/contact_points.csv
```

### 4. 测试系统

**使用测试脚本验证系统**
```bash
# 终端1：启动接触点记录器
ros2 run fr3_control_publish contact_point_recorder

# 终端2：运行测试脚本
python3 src/fr3_control_publish/scripts/test_contact_recording.py
```

## 输出文件

- `./contact_data/contact_points.csv` - 接触点数据
- `./contact_data/contact_points.ply` - PLY格式点云

## 参数调整

可以通过ROS2参数调整系统行为：

```bash
# 调整接触力阈值
ros2 run fr3_control_publish contact_point_recorder --ros-args -p force_threshold:=3.0

# 调整输出目录
ros2 run fr3_control_publish contact_point_recorder --ros-args -p output_directory:=./my_contact_data

# 禁用可视化
ros2 run fr3_control_publish contact_point_recorder --ros-args -p enable_visualization:=false
```

## 故障排除

1. **检查话题连接**
```bash
ros2 topic list
ros2 topic echo /contact_points
```

2. **检查节点状态**
```bash
ros2 node list
ros2 node info /contact_point_recorder
```

3. **查看日志**
```bash
ros2 run fr3_control_publish contact_point_recorder --ros-args --log-level debug
``` 