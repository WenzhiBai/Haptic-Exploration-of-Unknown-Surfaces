// src/fr3_jacobian_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <mujoco/mujoco.h>

#include <vector>
#include <string>
#include <stdexcept>

class JacobianPublisher : public rclcpp::Node {
public:
  JacobianPublisher()
  : Node("jacobian_publisher")
  {
    // 1. 声明并读取参数
    this->declare_parameter<std::string>(
      "model_xml",
      "/home/mscrobotics2425laptop30/yxc/Project_HEUS/ros2_mujoco_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/fr3.xml"  // 修改为你的 fr3.xml 路径
    );
    this->declare_parameter<std::string>(
      "end_effector_site",
      "attachment_site"  // 默认为 fr3.xml 中定义的 <site name="attachment_site"/>
    );
    model_path_ = this->get_parameter("model_xml").as_string();
    site_name_  = this->get_parameter("end_effector_site").as_string();

    // 2. 加载 MuJoCo 模型
    char error[1000] = "Could not load model";
    m_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    if (!m_) {
      RCLCPP_FATAL(this->get_logger(),
                   "加载模型失败 '%s': %s", model_path_.c_str(), error);
      throw std::runtime_error("Failed to load MuJoCo model");
    }
    d_ = mj_makeData(m_);
    nv_ = m_->nv;

    // 3. 查 site_id
    site_id_ = mj_name2id(m_, mjOBJ_SITE, site_name_.c_str());
    if (site_id_ < 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Site '%s' not found in model", site_name_.c_str());
      throw std::runtime_error("Invalid end_effector_site");
    }

    // 4. 预分配雅各比缓存
    jacp_.resize(3 * nv_);
    jacr_.resize(3 * nv_);

    // 5. 订阅 joint_states，发布 jacobian
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JacobianPublisher::on_joint_state, this, std::placeholders::_1)
    );
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/jacobian", 10
    );

    RCLCPP_INFO(this->get_logger(),
                "JacobianPublisher 启动，模型='%s'，site='%s'",
                model_path_.c_str(), site_name_.c_str());


    // 预分配完整补偿力矩缓存
    gravity_compensation_.resize(nv_);

    // 新增补偿力矩发布器
    gravity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gravity_compensation", 10
    );



    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/ee_pose", rclcpp::SensorDataQoS());



    sensor_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/sensor_wrench", rclcpp::SensorDataQoS());

  }

  ~JacobianPublisher() override {
    mj_deleteData(d_);
    mj_deleteModel(m_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) 
  {
    // 更新 qpos
    for (size_t i = 0; i < msg->name.size(); ++i) {
      int jid = mj_name2id(m_, mjOBJ_JOINT, msg->name[i].c_str());
      if (jid < 0) continue;
      int qadr = m_->jnt_qposadr[jid];
      d_->qpos[qadr] = msg->position[i];
    }

    // 前向运动学
    mj_forward(m_, d_);

    // 计算雅各比：线速度 jacp_，角速度 jacr_
    mj_jacSite(m_, d_, jacp_.data(), jacr_.data(), site_id_);

    // 拼接 6×nv
    std_msgs::msg::Float64MultiArray out;
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows";
    out.layout.dim[0].size  = 6;
    out.layout.dim[1].label = "cols";
    out.layout.dim[1].size  = nv_;
    out.data.reserve(6 * nv_);
    // 线速度部分
    for (double v : jacp_) out.data.push_back(v);
    // 角速度部分
    for (double v : jacr_) out.data.push_back(v);

    //发布雅可比矩阵
    pub_->publish(std::move(out));

    //计算并发布末端位姿
    publishEndEffectorPose();

    //计算并发布传感器数据
    publishSensorWrench();


    /*****************************************************************************************/

    // 计算重力补偿
    compute_gravity_compensation();

    // 发布重力补偿力矩
    std_msgs::msg::Float64MultiArray gravity_msg;
    gravity_msg.layout.dim.resize(1);
    gravity_msg.layout.dim[0].label = "gravity_compensation";
    gravity_msg.layout.dim[0].size = nv_;
    gravity_msg.data.assign(gravity_compensation_.begin(), gravity_compensation_.end());
    gravity_pub_->publish(gravity_msg);
    
    /*****************************************************************************************/

  }

  void publishSensorWrench()
  {
      // 获取 sensor ID
      int force_id = mj_name2id(m_, mjOBJ_SENSOR, "sensor_force");
      int torque_id = mj_name2id(m_, mjOBJ_SENSOR, "sensor_torque");

      if (force_id < 0 || torque_id < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Sensor 'sensor_force' or 'sensor_torque' not found!");
        return;
      }

      // 获取 sensor 索引偏移
      int force_offset = m_->sensor_adr[force_id];
      int torque_offset = m_->sensor_adr[torque_id];

      // 提取力和力矩
      const double* f = d_->sensordata + force_offset;
      const double* t = d_->sensordata + torque_offset;

      // 构造 WrenchStamped 消息
      geometry_msgs::msg::WrenchStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "touch_tip";  // 或你希望用的 frame

      msg.wrench.force.x = f[0];
      msg.wrench.force.y = f[1];
      msg.wrench.force.z = f[2];

      msg.wrench.torque.x = t[0];
      msg.wrench.torque.y = t[1];
      msg.wrench.torque.z = t[2];

      sensor_wrench_pub_->publish(msg);
  }

  // 计算重力补偿（仅重力，因为使用直接力控制）
  void compute_gravity_compensation() 
  {
    // 保存当前状态
    std::vector<double> qpos_backup(d_->qpos, d_->qpos + m_->nq);
    std::vector<double> qvel_backup(d_->qvel, d_->qvel + m_->nv);
    std::vector<double> qacc_backup(d_->qacc, d_->qacc + m_->nv);

    // 设置零速度和零加速度（仅计算重力项）
    std::fill(d_->qvel, d_->qvel + m_->nv, 0.0);
    std::fill(d_->qacc, d_->qacc + m_->nv, 0.0);

    // 计算逆向动力学（仅重力项）
    // flg_act=0 表示只计算重力项，不包括执行器力
    mj_rne(m_, d_, 0, gravity_compensation_.data());

    // 重力补偿应该是负的，因为我们要抵消重力
    for (int i = 0; i < nv_; ++i) {
      gravity_compensation_[i] = gravity_compensation_[i];
    }

    // 恢复状态
    std::copy(qpos_backup.begin(), qpos_backup.end(), d_->qpos);
    std::copy(qvel_backup.begin(), qvel_backup.end(), d_->qvel);
    std::copy(qacc_backup.begin(), qacc_backup.end(), d_->qacc);
  }

  void publishEndEffectorPose()
{
    // 获取site的位置和姿态
    // 使用site的body ID来获取位置和姿态
    int body_id = m_->site_bodyid[site_id_];
    
    // 获取body的位置和姿态
    const double* pos = d_->xpos + 3 * body_id;
    const double* quat = d_->xquat + 4 * body_id;
    
    // 获取site相对于body的偏移
    const double* site_offset = m_->site_pos + 3 * site_id_;
    
    // 计算site的世界坐标位置
    mjtNum site_pos[3];
    site_pos[0] = pos[0] + site_offset[0];
    site_pos[1] = pos[1] + site_offset[1];
    site_pos[2] = pos[2] + site_offset[2];

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "fr3_link0";  // 或者其他你认为的 base_frame

    pose_msg.pose.position.x = site_pos[0];
    pose_msg.pose.position.y = site_pos[1];
    pose_msg.pose.position.z = site_pos[2];

    // 对于site，通常使用body的姿态
    pose_msg.pose.orientation.w = quat[0];
    pose_msg.pose.orientation.x = quat[1];
    pose_msg.pose.orientation.y = quat[2];
    pose_msg.pose.orientation.z = quat[3];

    ee_pose_pub_->publish(pose_msg);
}


  // MuJoCo
  mjModel* m_{nullptr};
  mjData*  d_{nullptr};
  int      nv_{0};
  int      site_id_{-1};
  std::string model_path_, site_name_;
  std::vector<double> jacp_, jacr_;

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  // 重力补偿
  std::vector<double> gravity_compensation_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_pub_;

  //末端位置
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;


  //传感器值
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_wrench_pub_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<JacobianPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                 "节点启动失败: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}