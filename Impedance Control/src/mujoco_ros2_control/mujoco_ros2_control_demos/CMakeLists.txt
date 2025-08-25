cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
#find_package(Mujoco REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED) 

# =========================================================================
# 2. 生成自定义消息接口 (Generate custom message interfaces)
#    这一步必须在所有依赖这些消息的可执行文件定义之前
# =========================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointStateArray.msg" # 指定你的自定义消息文件
  DEPENDENCIES std_msgs
  # 如果有其他消息，可以在这里添加
)

# 添加头文件搜索路径（在所有 target 之前）
include_directories(
  ${kdl_parser_INCLUDE_DIRS}      # 新增
  ${orocos_kdl_INCLUDE_DIRS}      # 新增
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
)

# =========================================================================
# 3. 定义包的通用依赖列表 (Define common package dependency list)
# =========================================================================
set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
 # Mujoco
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  trajectory_msgs
  kdl_parser
  orocos_kdl
  tf2
  tf2_eigen
)

install(DIRECTORY
  launch
  config
  urdf
  mujoco_models
  msg
  DESTINATION share/${PROJECT_NAME}/
)


add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position ${THIS_PACKAGE_DEPENDS})

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity ${THIS_PACKAGE_DEPENDS})

add_executable(example_effort examples/example_effort.cpp)
ament_target_dependencies(example_effort ${THIS_PACKAGE_DEPENDS})

add_executable(example_diff_drive examples/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive ${THIS_PACKAGE_DEPENDS})

add_executable(example_tricycle_drive examples/example_tricycle_drive.cpp)
ament_target_dependencies(example_tricycle_drive ${THIS_PACKAGE_DEPENDS})

add_executable(example_gripper examples/example_gripper.cpp)
ament_target_dependencies(example_gripper ${THIS_PACKAGE_DEPENDS})

add_executable(example_camera examples/example_camera.cpp)
ament_target_dependencies(example_camera ${THIS_PACKAGE_DEPENDS})

add_executable(fr3_mujoco_bridge_node examples/fr3_mujoco_bridge_node.cpp)
ament_target_dependencies(fr3_mujoco_bridge_node ${THIS_PACKAGE_DEPENDS})



# Manually specify Mujoco include directories
target_include_directories(fr3_mujoco_bridge_node PUBLIC
    /opt/mujoco/include # Mujoco headers path
    # Add any other necessary include paths if they are not covered by THIS_PACKAGE_DEPENDS
)

# Manually specify Mujoco link libraries
target_link_libraries(fr3_mujoco_bridge_node
    /opt/mujoco/lib/libmujoco.so # Direct path to Mujoco library
    # If your system needs it, you might also need other standard libraries like GL/GLU/GLEW/GLFW for visualization, but for pure control/physics, libmujoco.so is key.
)


# (State Manager Node)
add_executable(state_manager_node examples/state_manager_node.cpp)
ament_target_dependencies(state_manager_node ${THIS_PACKAGE_DEPENDS})
target_link_libraries(state_manager_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)


add_executable(fr3_impedance_controller examples/fr3_impedance_controller.cpp)
ament_target_dependencies(fr3_impedance_controller ${THIS_PACKAGE_DEPENDS})
target_link_libraries(fr3_impedance_controller
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${kdl_parser_LIBRARIES}      # 
  ${orocos_kdl_LIBRARIES}      # 
)

install(TARGETS
    example_position
    example_velocity
    example_effort
    example_diff_drive
    example_tricycle_drive
    example_gripper
    example_camera
    fr3_mujoco_bridge_node
    state_manager_node
    fr3_impedance_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
