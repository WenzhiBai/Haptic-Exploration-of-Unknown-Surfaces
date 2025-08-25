import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享目录
    demo_pkg_path = get_package_share_directory('mujoco_ros2_control_demos')

    # 定义 Mujoco 模型 路径
    mujoco_model_path = os.path.join(demo_pkg_path, 'mujoco_models', 'scene.xml')

    print(f"Mujoco model path: {mujoco_model_path}") # 可选：用于调试路径解析

    # FR3 Mujoco 桥接节点
    fr3_mujoco_bridge_node = Node(
        package='mujoco_ros2_control_demos', # 包名
        executable='fr3_mujoco_bridge_node', # 在 CMakeLists.txt 中定义的可执行文件名
        name='fr3_mujoco_bridge', # 可选：节点的自定义名称
        output='screen', # 在控制台显示节点输出
        parameters=[
            {'mujoco_model_path': mujoco_model_path}, # 将模型路径作为参数传递给桥接 C++ 节点
            {'use_sim_time': True} # 对仿真节点通常很有用
        ]
    )

    return LaunchDescription([
        fr3_mujoco_bridge_node,
    ])
