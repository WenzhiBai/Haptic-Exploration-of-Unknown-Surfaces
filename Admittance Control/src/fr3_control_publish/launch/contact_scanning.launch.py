#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('fr3_control_publish')
    
    # 声明启动参数
    declare_output_dir = DeclareLaunchArgument(
        'output_directory',
        default_value='./contact_data',
        description='接触点数据输出目录'
    )
    
    declare_force_threshold = DeclareLaunchArgument(
        'force_threshold',
        default_value='5.0',
        description='接触力阈值 (N)'
    )
    
    declare_enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='是否启用可视化'
    )
    
    # 创建节点
    contact_recorder_node = Node(
        package='fr3_control_publish',
        executable='contact_point_recorder',
        name='contact_point_recorder',
        parameters=[{
            'output_directory': LaunchConfiguration('output_directory'),
            'force_threshold': LaunchConfiguration('force_threshold'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'ee_pose_topic': '/ee_pose',
            'wrench_topic': '/touch_tip/wrench',
            'contact_points_topic': '/contact_points'
        }],
        output='screen'
    )
    
    # 创建启动描述
    return LaunchDescription([
        declare_output_dir,
        declare_force_threshold,
        declare_enable_visualization,
        contact_recorder_node
    ]) 