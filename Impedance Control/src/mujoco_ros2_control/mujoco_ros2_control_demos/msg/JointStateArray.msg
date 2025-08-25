#/msg/JointStateArray.msg

std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort


#string force_torque_sensor_name # 力/力矩传感器的名称，例如 "touch_tip_sensor"
#float64[3] force                # 三维力向量 (Fx, Fy, Fz)，单位：牛顿 (N)
#float64[3] torque               # 三维力矩向量 (Tx, Ty, Tz)，单位：牛米 (Nm)
