#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_ros2_control_demos')

    # ---------- 1. 机器人 URDF ----------
    urdf_path = os.path.join(pkg_share, 'urdf', 'fr3_nohand.urdf')
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # ---------- 2. MuJoCo-ros2_control ----------
    controller_yaml = os.path.join(pkg_share, 'config', 'fr3_controllers.yaml')
    scene_xml       = os.path.join(pkg_share, 'mujoco_models', 'scene.xml')

    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[robot_description,
                    controller_yaml,
                    {'mujoco_model_path': scene_xml},]
    )

    # ---------- 3. Robot State Publisher ----------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ---------- 4) 直接启动控制器节点 ----------
    # 启动joint_state_broadcaster
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    
    # 启动joint_position_controller
    joint_position_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_position_controller_spawner',
        output='screen',
        arguments=['joint_position_controller', '--controller-manager', '/controller_manager']
    )

    # 延迟启动控制器节点
    delayed_controllers = TimerAction(
        period=3.0,  # 延迟3秒启动控制器
        actions=[
            joint_state_broadcaster_node,
            joint_position_controller_node
        ]
    )

    # ---------- 5. 接触点记录系统节点 ----------
    # 雅可比发布器
    jacobian_publisher_node = Node(
        package='fr3_control_publish',
        executable='jacobian_publisher',
        name='jacobian_publisher',
        output='screen',
        parameters=[{
            'model_xml': '/home/mscrobotics2425laptop16/mujoco_ros_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/fr3.xml',
            'end_effector_site': 'attachment_site'
        }]
    )

    # 末端位姿桥接器
    ee_pose_bridge_node = Node(
        package='fr3_control_publish',
        executable='ee_pose_bridge',
        name='ee_pose_bridge',
        output='screen',
        parameters=[{
            'base_frame': 'fr3_link0',
            'ee_frame': 'touch_tip',
            'output_topic': '/ee_pose',
            'publish_rate': 100.0
        }]
    )

    # 接触点记录器
    contact_point_recorder_node = Node(
        package='fr3_control_publish',
        executable='contact_point_recorder',
        name='contact_point_recorder',
        output='screen',
        parameters=[{
            'output_directory': './contact_data',
            'force_threshold': 5.0,
            'min_contact_duration': 0.1,
            'enable_visualization': True,
            'ee_pose_topic': '/ee_pose',
            'wrench_topic': '/touch_tip/wrench',
            'contact_points_topic': '/contact_points'
        }]
    )

    # 混合力控制控制器
    hybrid_force_controller_node = Node(
        package='fr3_control_publish',
        executable='hybridforcetangentialcontroller',
        name='hybrid_force_tangential_controller',
        output='screen',
        parameters=[{
            # 力控制参数
            'f_high': 25.0,                    # 接触阈值 (N)
            'f_low': 0.5,                      # 离开阈值 (N)
            'desired_contact_force': 25.0,     # 期望接触力 (N)
            
            # 扫描步长参数
            'step_length': 0.003,               # 主轴推进步长 (m) - 1cm，可在launch文件中调整
            'lateral_step': 0.005,              # 横向扫描步长 (m) - 5mm，提高扫描密度
            
            # 速度限制参数
            'axis_speed': 0.02,                # 主轴推进速度 (m/s)
            'lateral_speed': 0.1,              # 横向扫描速度 (m/s)
            'init_position_speed': 0.05,       # 初始化位置控制速度 (m/s)
            
            # 姿态控制参数
            'theta_tol_deg': 2.0,              # 姿态对齐容差 (度)
            'k_theta': 0.2,                    # 姿态微旋系数
            'max_angular_velocity': 0.5,       # 最大角速度限制 (rad/s)
            'orientation_gain': 5.0,           # 姿态控制增益
            'position_tolerance': 0.001,       # 位置控制容差 (m) - 1mm，提高推进精度
            
            # 初始化目标位置
            'init_target_x': 0.7,              # 初始化目标X坐标 (m)
            'init_target_y': 0.0,              # 初始化目标Y坐标 (m)
            'init_target_z': 0.15,             # 初始化目标Z坐标 (m)
            'init_position_tolerance': 0.005,  # 初始化位置容差 (m)
            
            # 物体参数
            'object_center_x': 0.65,           # 物体中心X坐标 (m)
            'object_center_y': 0.0,            # 物体中心Y坐标 (m)
            'object_width': 0.2,               # 物体宽度 (m)
            'object_height': 0.3,              # 物体高度 (m)
            
            # 测力参数
            'T_pause': 0.15,                   # 静态测力窗口时长 (s)
            'N_min': 10,                       # 窗口内最少采样帧数
            'f_eps': 0.2,                      # 力波动阈值 (N)
            
            # 话题配置
            'ee_pose_topic': '/ee_pose',       # 末端位姿话题
        }]
    )

    # 延迟启动接触点记录系统（等待基础系统启动完成）
    delayed_contact_system = TimerAction(
        period=5.0,  # 延迟5秒启动
        actions=[
            jacobian_publisher_node,
            ee_pose_bridge_node,
            contact_point_recorder_node,
            hybrid_force_controller_node
        ]
    )

    return LaunchDescription([
        RegisterEventHandler(OnProcessStart(
            target_action=mujoco_node,
            on_start=[delayed_controllers])),
        mujoco_node,
        rsp_node,
        delayed_contact_system,  # 添加延迟启动的接触点记录系统
    ])
