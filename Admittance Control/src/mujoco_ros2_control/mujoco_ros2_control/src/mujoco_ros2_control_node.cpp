#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

#include "mujoco/mjvisualize.h"

#include <cstring>
#include <cmath> // For std::abs

// MuJoCo data structures
static mjModel* mujoco_model = nullptr;
static mjData*  mujoco_data  = nullptr;

int main(int argc, const char** argv)
{
  // ----------------------------- ROS 2 初始化 -----------------------------
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // RCLCPP_INFO(logger, "Initializing mujoco_ros2_control node ...");
  std::string model_path = node->get_parameter("mujoco_model_path").as_string();
  
  // 获取接触检测相关参数
  double force_threshold = node->get_parameter_or("force_threshold", 1e-6);
  // Z轴力阈值：小于此值认为无接触（默认-0.1N，允许轻微的负力）
  double z_force_threshold = node->get_parameter_or("z_force_threshold", -0.1);
  // 法向力提取模式：0=Z轴方向，1=接触法向，2=自定义方向
  int normal_force_mode = node->get_parameter_or("normal_force_mode", 0);
  // 自定义法向方向（当mode=2时使用）
  std::vector<double> custom_normal = node->get_parameter_or("custom_normal_direction", std::vector<double>{0.0, 0.0, -1.0});
  
  // RCLCPP_INFO(logger, "力检测参数:");
  // RCLCPP_INFO(logger, "  force_threshold: %.2e", force_threshold);
  // RCLCPP_INFO(logger, "  z_force_threshold: %.3f N", z_force_threshold);
  // RCLCPP_INFO(logger, "  normal_force_mode: %d", normal_force_mode);
  // if (normal_force_mode == 2) {
  //   RCLCPP_INFO(logger, "  custom_normal_direction: [%.3f, %.3f, %.3f]", 
  //                custom_normal[0], custom_normal[1], custom_normal[2]);
  // }

  // ----------------------------- 加载 MuJoCo 模型 -----------------------------
  char error[1000] = "Could not load binary model";
  if (model_path.size() > 4 &&
      model_path.compare(model_path.size()-4, 4, ".mjb") == 0)
  {
    mujoco_model = mj_loadModel(model_path.c_str(), nullptr);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  }
  if (!mujoco_model)
    mju_error("Load model error: %s", error);

  // RCLCPP_INFO(logger, "MuJoCo model successfully loaded");
  mujoco_data = mj_makeData(mujoco_model);

  // ----------------------------- 获取传感器 ID 和几何体 ID -----------------------------
  int id_force_sensor = mj_name2id(mujoco_model, mjOBJ_SENSOR, "touch_tip_force");
  int id_torque_sensor = mj_name2id(mujoco_model, mjOBJ_SENSOR, "touch_tip_torque");
  
  // 获取碰撞几何体ID用于接触检测
  int id_ee = mj_name2id(mujoco_model, mjOBJ_GEOM, "touch_tip_collision");
  int id_ee_backup = mj_name2id(mujoco_model, mjOBJ_GEOM, "touch_tip_collision_backup");
  int id_box = mj_name2id(mujoco_model, mjOBJ_GEOM, "exploration_object_collision");
  
  // 验证传感器ID
  if (id_force_sensor < 0) {
    RCLCPP_ERROR(logger, "找不到力传感器 'touch_tip_force'");
  } else {
    RCLCPP_INFO(logger, "力传感器 'touch_tip_force' ID: %d", id_force_sensor);
  }
  
  if (id_torque_sensor < 0) {
    RCLCPP_ERROR(logger, "找不到力矩传感器 'touch_tip_torque'");
  } else {
    RCLCPP_INFO(logger, "力矩传感器 'touch_tip_torque' ID: %d", id_torque_sensor);
  }
  
  // 验证几何体ID
  if (id_ee < 0 && id_ee_backup < 0) {
    RCLCPP_ERROR(logger, "找不到末端执行器碰撞几何体");
  }
  
  if (id_box < 0) {
    RCLCPP_ERROR(logger, "找不到探索物体碰撞几何体");
  }
  
  if (id_force_sensor < 0 || id_torque_sensor < 0 || 
      (id_ee < 0 && id_ee_backup < 0) || id_box < 0) {
    RCLCPP_ERROR(logger, "无法找到必要的传感器或几何体，力检测可能失败");
  }

  // ----------------------------- ROS 2 发布器 -----------------------------
  auto wrench_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "touch_tip/wrench", rclcpp::SensorDataQoS());

  // ----------------------------- 初始化控制 & 渲染 -----------------------------
  mujoco_ros2_control::MujocoRos2Control control(node, mujoco_model, mujoco_data);
  control.init();
  // RCLCPP_INFO(logger, "Mujoco ros2 controller initialized");

  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(node, mujoco_model, mujoco_data);
  // RCLCPP_INFO(logger, "Mujoco rendering initialized");
  // rendering->mjv_opt_.flags[mjVIS_CONTACTPOINT] = 1;
  // rendering->mjv_opt_.flags[mjVIS_CONTACTFORCE] = 1;

  // ----------------------------- 主循环 -----------------------------
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  {
    const mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0/60.0)
    {
      // 一步仿真（包含 mj_step1 + 控制器 + mj_step2）
      control.update();

      // --- 使用传感器读取力数据并提取真正的法向力 ---
      mjtNum force_world[3] = {0}, torque_world[3] = {0};
      bool have_sensor_data = false;
      bool have_contact_normal = false;
      bool z_force_negative = false;  // 移到外层，确保作用域覆盖
      std::vector<double> contact_normal(3, 0.0);
      
      // 1. 从传感器读取原始力数据
      if (id_force_sensor >= 0 && id_torque_sensor >= 0) {
        int force_adr = mujoco_model->sensor_adr[id_force_sensor];
        int torque_adr = mujoco_model->sensor_adr[id_torque_sensor];
        
        if (force_adr >= 0 && force_adr + 2 < mujoco_model->nsensordata) {
          force_world[0] = mujoco_data->sensordata[force_adr + 0];  // X方向力
          force_world[1] = mujoco_data->sensordata[force_adr + 1];  // Y方向力
          force_world[2] = mujoco_data->sensordata[force_adr + 2];  // Z方向力
          have_sensor_data = true;
          
          // 检查Z轴力是否为负数（无接触状态）
          double z_force = mujoco_data->sensordata[force_adr + 2];  // Z方向力
          z_force_negative = (z_force < z_force_threshold); // 使用可配置的阈值
          
          // 调试输出Z轴力状态
          static int z_force_debug_counter = 0;
          if (z_force_debug_counter++ % 200 == 0) {
            // RCLCPP_INFO(logger, "Z轴力检测: %.6g N, 状态: %s", 
            //             z_force, z_force_negative ? "无接触(负力)" : "有接触(正力)");
          }
        }
        
        if (torque_adr >= 0 && torque_adr + 2 < mujoco_model->nsensordata) {
          torque_world[0] = mujoco_data->sensordata[torque_adr + 0];  // X方向力矩
          torque_world[1] = mujoco_data->sensordata[torque_adr + 1];  // Y方向力矩
          torque_world[2] = mujoco_data->sensordata[torque_adr + 2];  // Z方向力矩
        }
      }
      
      // 2. 通过接触检测获取接触面法向量
      if (have_sensor_data && (id_ee >= 0 || id_ee_backup >= 0) && id_box >= 0) {
        // 如果Z轴力为负数，认为无接触，跳过接触检测
        if (z_force_negative) {
          have_contact_normal = false;
          // 清零接触法向量
          std::fill(contact_normal.begin(), contact_normal.end(), 0.0);
        } else {
          // Z轴力为正数，进行正常的接触检测
          if (mujoco_data->ncon > 0) {
            for (int i = 0; i < mujoco_data->ncon; ++i) {
              auto& con = mujoco_data->contact[i];
              
              // 检查是否是目标接触对（EE和Box之间）
              bool is_target_contact = false;
              if ((con.geom1 == id_ee && con.geom2 == id_box) ||
                  (con.geom1 == id_box && con.geom2 == id_ee) ||
                  (con.geom1 == id_ee_backup && con.geom2 == id_box) ||
                  (con.geom1 == id_box && con.geom2 == id_ee_backup)) {
                is_target_contact = true;
              }
              
              if (is_target_contact) {
                // 获取接触面法向量（contact frame的第一列）
                // 注意：MuJoCo的contact frame中，第一列是法向量
                contact_normal[0] = con.frame[0];  // 法向量X分量
                contact_normal[1] = con.frame[1];  // 法向量Y分量
                contact_normal[2] = con.frame[2];  // 法向量Z分量
                
                // 归一化法向量
                double norm = std::sqrt(contact_normal[0]*contact_normal[0] + 
                                      contact_normal[1]*contact_normal[1] + 
                                      contact_normal[2]*contact_normal[2]);
                if (norm > 1e-6) {
                  contact_normal[0] /= norm;
                  contact_normal[1] /= norm;
                  contact_normal[2] /= norm;
                  have_contact_normal = true;
                  
                  // 调试输出接触法向量
                  static int normal_debug_counter = 0;
                  if (normal_debug_counter++ % 200 == 0) {
                    // RCLCPP_INFO(logger, "检测到接触，法向量: [%.3f, %.3f, %.3f]", 
                    //             contact_normal[0], contact_normal[1], contact_normal[2]);
                  }
                  break;  // 找到第一个有效接触就退出
                }
              }
            }
          }
        }
      }
      
      // 3. 根据法向力提取模式处理力数据
      if (have_sensor_data) {
        double force_magnitude = std::sqrt(force_world[0]*force_world[0] + 
                                         force_world[1]*force_world[1] + 
                                         force_world[2]*force_world[2]);
        
        if (force_magnitude > force_threshold) {
          double normal_force = 0.0;
          double tangential_force = 0.0;
          std::vector<double> final_normal_direction(3, 0.0);
          
          // 根据模式选择法向方向
          switch (normal_force_mode) {
            case 0:  // 自动检测接触面法向（推荐）
              if (have_contact_normal && !z_force_negative) {
                // 使用检测到的接触面法向量
                final_normal_direction = contact_normal;
                // 计算力在法向量上的投影
                normal_force = force_world[0]*contact_normal[0] + 
                              force_world[1]*contact_normal[1] + 
                              force_world[2]*contact_normal[2];
                // 计算切向力
                tangential_force = std::sqrt(force_magnitude*force_magnitude - normal_force*normal_force);
              } else {
                // 没有检测到接触或Z轴力为负数，使用Z轴方向作为备选
                if (z_force_negative) {
                  // RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000, 
                  //   "Z轴力为负数，认为无接触，使用Z轴方向作为法向");
                } else {
                  // RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000, 
                  //   "未检测到接触，使用Z轴方向作为法向");
                }
                final_normal_direction = {0.0, 0.0, 1.0};
                normal_force = force_world[2];
                tangential_force = std::sqrt(force_world[0]*force_world[0] + force_world[1]*force_world[1]);
              }
              break;
              
            case 1:  // 固定Z轴方向
              final_normal_direction = {0.0, 0.0, 1.0};
              normal_force = force_world[2];
              tangential_force = std::sqrt(force_world[0]*force_world[0] + force_world[1]*force_world[1]);
              break;
              
            case 2:  // 自定义方向
              if (custom_normal.size() == 3) {
                double norm = std::sqrt(custom_normal[0]*custom_normal[0] + 
                                      custom_normal[1]*custom_normal[1] + 
                                      custom_normal[2]*custom_normal[2]);
                if (norm > 1e-6) {
                  final_normal_direction[0] = custom_normal[0] / norm;
                  final_normal_direction[1] = custom_normal[1] / norm;
                  final_normal_direction[2] = custom_normal[2] / norm;
                  
                  normal_force = force_world[0]*final_normal_direction[0] + 
                                force_world[1]*final_normal_direction[1] + 
                                force_world[2]*final_normal_direction[2];
                  tangential_force = std::sqrt(force_magnitude*force_magnitude - normal_force*normal_force);
                } else {
                  RCLCPP_WARN(logger, "自定义法向方向为零向量，使用Z轴方向");
                  final_normal_direction = {0.0, 0.0, 1.0};
                  normal_force = force_world[2];
                  tangential_force = std::sqrt(force_world[0]*force_world[0] + force_world[1]*force_world[1]);
                }
              } else {
                RCLCPP_WARN(logger, "自定义法向方向参数错误，使用Z轴方向");
                final_normal_direction = {0.0, 0.0, 1.0};
                normal_force = force_world[2];
                tangential_force = std::sqrt(force_world[0]*force_world[0] + force_world[1]*force_world[1]);
              }
              break;
              
            default:
              RCLCPP_WARN(logger, "未知的法向力模式 %d，使用自动检测", normal_force_mode);
              if (have_contact_normal) {
                final_normal_direction = contact_normal;
                normal_force = force_world[0]*contact_normal[0] + 
                              force_world[1]*contact_normal[1] + 
                              force_world[2]*contact_normal[2];
                tangential_force = std::sqrt(force_magnitude*force_magnitude - normal_force*normal_force);
              } else {
                final_normal_direction = {0.0, 0.0, 1.0};
                normal_force = force_world[2];
                tangential_force = std::sqrt(force_world[0]*force_world[0] + force_world[1]*force_world[1]);
              }
              break;
          }
          
          // 4. 重新计算力分量：只保留法向力
          force_world[0] = normal_force * final_normal_direction[0];
          force_world[1] = normal_force * final_normal_direction[1];
          force_world[2] = normal_force * final_normal_direction[2];
          
          // 调试输出
          static int debug_counter = 0;
          if (debug_counter++ % 100 == 0) {
            // RCLCPP_INFO(logger, 
            //   "力数据: 原始力=[%.6g,%.6g,%.6g] N, 切向力=%.6g N, 法向力=%.6g N",
            //   mujoco_data->sensordata[mujoco_model->sensor_adr[id_force_sensor] + 0], 
            //   mujoco_data->sensordata[mujoco_model->sensor_adr[id_force_sensor] + 1], 
            //   mujoco_data->sensordata[mujoco_model->sensor_adr[id_force_sensor] + 2],
            //   tangential_force, normal_force);
            // RCLCPP_INFO(logger, 
            //   "过滤后: force=[%.6g,%.6g,%.6g] N, 法向方向=[%.3f,%.3f,%.3f], 接触检测=%s, Z轴力状态=%s",
            //   force_world[0], force_world[1], force_world[2],
            //   final_normal_direction[0], final_normal_direction[1], final_normal_direction[2],
            //   have_contact_normal ? "是" : "否",
            //   z_force_negative ? "负力(无接触)" : "正力(有接触)");
          }
        } else {
          // 力太小，清零数据
          std::fill(force_world, force_world + 3, 0.0);
          std::fill(torque_world, torque_world + 3, 0.0);
        }
      } else {
        RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000, "传感器ID无效，无法读取力数据");
      }

      // --- 发布 WrenchStamped ---
      geometry_msgs::msg::WrenchStamped msg;
      msg.header.stamp    = node->get_clock()->now();
      msg.header.frame_id = "world";
      msg.wrench.force.x  = force_world[0];
      msg.wrench.force.y  = force_world[1];
      msg.wrench.force.z  = force_world[2];
      msg.wrench.torque.x = torque_world[0];
      msg.wrench.torque.y = torque_world[1];
      msg.wrench.torque.z = torque_world[2];
      wrench_pub->publish(msg);
    }

    rendering->update();  // 刷新渲染
  }

  // ----------------------------- 清理 -----------------------------
  rendering->close();
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);
  // RCLCPP_INFO(logger, "MuJoCo simulation terminated. Bye!");
  return 0;
}
