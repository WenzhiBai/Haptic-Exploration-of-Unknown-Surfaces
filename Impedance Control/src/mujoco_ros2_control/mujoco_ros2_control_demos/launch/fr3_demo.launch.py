import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # 1. 定位文件路径
    demo_pkg_path = get_package_share_directory('mujoco_ros2_control_demos')
    urdf_path = os.path.join(demo_pkg_path, 'urdf', 'fr3_nohand.urdf')
    mujoco_model_path = os.path.join(demo_pkg_path, 'mujoco_models', 'scene.xml')  
    controller_config_path = os.path.join(demo_pkg_path, 'config', 'fr3_controllers.yaml')
    rviz_config_path = os.path.join(demo_pkg_path, 'launch', 'fr3_demo.rviz')
    

    # 2. 加载URDF文件（直接使用URDF，无需xacro解析）
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 3. 配置节点
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        parameters=[
            {'robot_description': robot_description},
            {'mujoco_model_path': mujoco_model_path},
            {'use_sim_time': True},
            controller_config_path
        ],
        output='screen'
    )
    
    

    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    
    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )
        # ========== 新增的节点配置 ==========

    # 状态管理节点 StateManagerNode
    state_manager_node = Node(
        package='mujoco_ros2_control_demos', # 你的包名
        executable='state_manager_node', # 对应 CMakeLists.txt 中定义的可执行文件名
        name='state_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True}] # 状态管理节点通常也需要同步到仿真时间
    )
    
    
    # 状态管理节点 StateManagerNode
    fr3_mujoco_bridge_node = Node(
        package='mujoco_ros2_control_demos', # 你的包名
        executable='fr3_mujoco_bridge_node', # 对应 CMakeLists.txt 中定义的可执行文件名
        name='fr3_mujoco_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': True}] # 状态管理节点通常也需要同步到仿真时间
    )

    
    # FR3 阻抗控制器节点 
    # 请确保 executable 的名称与你在 CMakeLists.txt 中定义的一致

    fr3_controller_node = Node( # 将变量名改为更通用的 fr3_controller_node
        package='mujoco_ros2_control_demos', # 你的包名
        executable='fr3_impedance_controller', # 请根据你实际的可执行文件名称进行修改
                                                         # 例如：'fr3_impedance_controller' 或 'fr3_cartesian_position_controller'
        name='fr3_controller', # 节点名称
        output='screen',
        parameters=[{'robot_description': robot_description}] # 传递 robot_description 给控制器以便 KDL 解析
    )
    


    """
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    """
    
    # 4. 定义启动顺序
    return LaunchDescription([
        # 1. 首先启动 fr3_controller_node
        fr3_controller_node,
        
        # 2. 当 fr3_controller_node 启动后，启动 mujoco_node
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=fr3_controller_node,
                on_start=[mujoco_node]
            )
        ),
        
        # 3. 当 mujoco_node 启动后，启动 joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[joint_state_broadcaster]
            )
        ),
        
        # 4. 当 joint_state_broadcaster 启动后，启动其他节点
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=joint_state_broadcaster,
                on_start=[state_manager_node,fr3_mujoco_bridge_node,load_effort_controller
                ]
            )
        )
    ])