# mujoco_ros2_control_demos/msg/MujocoFullState.msg
std_msgs/Header header
string[] joint_name
float64[] position
float64[] velocity
float64[] jacobian_translation # 扁平化的 3xN 平移雅可比矩阵 (N 是自由度数量)
float64[] jacobian_rotation  # 扁平化的 3xN 旋转雅可比矩阵
