#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono> 
#include <limits>

using std::placeholders::_1;

// 扫描状态机各阶段枚举
enum class ScanState {
  INIT_SCAN,       // 初始化：沿目标中心方向的法向力导纳运动
  PAUSE_MEASURE,   // 停顿测量：冻结导纳，采集静态力
  ALIGN_ORIENT,    // 姿态对齐：围绕接触点纯旋转与法向对齐
  UNFREEZE,        // 解冻导纳：恢复B_inv并设定新目标力
  HORIZONTAL_SCAN, // 横向离散扫描：左右两侧边界记录
  CONTINUE_HORIZONTAL_SCAN, // 继续横向扫描（跳过对齐和解冻）
  ADVANCE_ON_AXIS, // 主轴离散推进：沿主轴步进
  DONE             // 扫描完成
};

class HybridForceTangentialController : public rclcpp::Node
{
public:
  HybridForceTangentialController()
  : Node("hybrid_force_tangential_controller"),
    state_(ScanState::INIT_SCAN), admittance_frozen_(false), loop_count_(0)
  {
    // ------------ 参数声明并注释 ------------
    declare_parameter<double>("f_high", 25.0);      // 接触阈值, 法向力 ≥ f_high 判定接触 (单位: N)
    declare_parameter<double>("f_low", 0.5);       // 离开阈值, 法向力 ≤ f_low 判定离开 (单位: N)
    declare_parameter<double>("step_length", 0.01); // 主轴离散步长 (单位: m)，默认1cm，可在launch文件中调整
    declare_parameter<double>("lateral_step", 0.01);// 横向离散步长 (单位: m)，增大步长
    declare_parameter<double>("axis_speed", 0.02);  // 主轴导纳限速 (单位: m/s)，提高速度
    declare_parameter<double>("lateral_speed", 0.1);// 横向导纳限速 (单位: m/s)，提高速度
    declare_parameter<double>("T_pause", 0.15);    // 静态测力窗口时长 (单位: s)
    declare_parameter<int>("N_min", 10);           // 窗口内最少采样帧数
    declare_parameter<double>("f_eps", 0.2);       // 力波动阈值, max-min<f_eps 认为稳定
    declare_parameter<double>("theta_tol_deg", 2.0);// 姿态对齐容差 (单位: deg)
    declare_parameter<double>("k_theta", 0.2);     // 姿态微旋系数, δθ=k_theta·θ
    declare_parameter<double>("max_angular_velocity", 0.5); // 最大角速度限制 (单位: rad/s)
    declare_parameter<double>("init_target_x", 0.7); // 初始化目标位置X坐标 (单位: m)
    declare_parameter<double>("init_target_y", 0.0); // 初始化目标位置Y坐标 (单位: m)
    declare_parameter<double>("init_target_z", 0.15); // 初始化目标位置Z坐标 (单位: m)
    declare_parameter<double>("init_position_tolerance", 0.005); // 初始化位置容差 (单位: m)
    declare_parameter<double>("init_position_speed", 0.05); // 初始化位置控制速度 (单位: m/s)，提高速度
    declare_parameter<double>("orientation_gain", 5.0); // 姿态控制增益，进一步增大以提高响应性
    declare_parameter<double>("position_tolerance", 0.001); // 位置控制容差 (单位: m)，默认1mm
    declare_parameter<double>("object_center_x", 0.3);// 物体中心X坐标 (单位: m)
    declare_parameter<double>("object_center_y", 0.0);// 物体中心Y坐标 (单位: m)
    declare_parameter<double>("object_width", 0.3);  // 物体宽度 (单位: m)
    declare_parameter<double>("object_height", 0.3); // 物体高度 (单位: m)
    declare_parameter<std::string>("ee_pose_topic", "/ee_pose"); // 末端位姿话题名
    declare_parameter<double>("desired_contact_force", 30.0); // 期望恒定接触力 (单位: N)

    // 假设你在 params 中也声明了一个长度 7 的 vector<double> joint_velocity_limits
    declare_parameter<std::vector<double>>("joint_velocity_limits", std::vector<double>(7, 1.0));
    std::vector<double> vel_limits;
    get_parameter("joint_velocity_limits", vel_limits);
    for (int i = 0; i < 7; ++i) {
      qdot_max_(i) = vel_limits[i];
    }

    // ------------ 参数获取 ------------
    get_parameter("f_high", f_high_);
    get_parameter("f_low", f_low_);
    get_parameter("step_length", step_length_);
    get_parameter("lateral_step", lateral_step_);
    get_parameter("axis_speed", axis_speed_);
    get_parameter("lateral_speed", lateral_speed_);
    get_parameter("T_pause", T_pause_);
    get_parameter("N_min", N_min_);
    get_parameter("f_eps", f_eps_);
    double theta_tol_deg;
    get_parameter("theta_tol_deg", theta_tol_deg);
    theta_tol_ = theta_tol_deg * M_PI / 180.0; // 转换为弧度
    get_parameter("k_theta", k_theta_);
    get_parameter("max_angular_velocity", max_angular_velocity_);
    get_parameter("init_target_x", init_target_x_);
    get_parameter("init_target_y", init_target_y_);
    get_parameter("init_target_z", init_target_z_);
    get_parameter("init_position_tolerance", init_position_tolerance_);
    get_parameter("init_position_speed", init_position_speed_);
    get_parameter("orientation_gain", orientation_gain_);
    get_parameter("position_tolerance", position_tolerance_);
    double obj_x, obj_y;
    get_parameter("object_center_x", obj_x);
    get_parameter("object_center_y", obj_y);
    get_parameter("object_width", object_width_);
    get_parameter("object_height", object_height_);
    get_parameter("ee_pose_topic", ee_pose_topic_);
    get_parameter("desired_contact_force", desired_contact_force_);

    RCLCPP_INFO(get_logger(), "参数: f_high=%.2f, f_low=%.2f, step_length=%.3f, lateral_step=%.3f", 
                f_high_, f_low_, step_length_, lateral_step_);
    RCLCPP_INFO(get_logger(), "参数: axis_speed=%.3f, lateral_speed=%.3f, T_pause=%.3f", 
                axis_speed_, lateral_speed_, T_pause_);
    RCLCPP_INFO(get_logger(), "参数: theta_tol=%.3f rad, k_theta=%.3f, max_angular_velocity=%.3f rad/s", 
                theta_tol_, k_theta_, max_angular_velocity_);
    RCLCPP_INFO(get_logger(), "初始化目标位置: (%.3f, %.3f, %.3f), 容差=%.3f m, 速度=%.3f m/s", 
                init_target_x_, init_target_y_, init_target_z_, init_position_tolerance_, init_position_speed_);
    RCLCPP_INFO(get_logger(), "姿态控制增益: %.3f", orientation_gain_);
    RCLCPP_INFO(get_logger(), "物体中心: (%.3f, %.3f)", obj_x, obj_y);
    RCLCPP_INFO(get_logger(), "物体尺寸: %.3f x %.3f m", object_width_, object_height_);
    RCLCPP_INFO(get_logger(), "期望恒定接触力: %.1f N", desired_contact_force_);

    // 初始化虚拟阻尼矩阵 (B_inv) 和期望力 f_des
    B_inv_ = Eigen::DiagonalMatrix<double,6>(
      1/5000.0,1/5000.0,1/5000.0,  // XYZ 方向阻尼倒数
      1/500.0,1/500.0,1/500.0      // RPY 方向阻尼倒数
    );
    f_des_.setZero();

    // 计算主轴与横向轴（XY 平面）
    Eigen::Vector2d base(0.0, 0.0);
    Eigen::Vector2d center(obj_x, obj_y);
    main_axis_    = (center - base).normalized();
    lateral_axis_ = Eigen::Vector2d(-main_axis_.y(), main_axis_.x());
    object_center_.x() = obj_x;
    object_center_.y() = obj_y;

    // 获取物体参数
    this->get_parameter("object_center_x", object_center_.x());
    this->get_parameter("object_center_y", object_center_.y());
    this->get_parameter("object_width", object_width_);
    this->get_parameter("object_height", object_height_);
    
    // 修正物体中心位置，使其与MuJoCo中的实际位置匹配
    // 根据新的边界要求：X[0.55, 0.75], Y[-0.15, 0.15]
    // 物体中心应该在(0.65, 0.0)，宽度为0.2m，高度为0.3m
    object_center_ = Eigen::Vector2d(0.65, 0.0);  // 相对于fr3_link0的位置

    // 设置新的物体边界
    object_left_   = 0.55;   // 左边界
    object_right_  = 0.70;   // 右边界
    object_bottom_ = -0.15;  // 下边界
    object_top_    = 0.15;   // 上边界

    RCLCPP_INFO_STREAM(get_logger(), "main_axis = [" << main_axis_.transpose() << "]");
    RCLCPP_INFO_STREAM(get_logger(), "lateral_axis = [" << lateral_axis_.transpose() << "]");
    RCLCPP_INFO(get_logger(), "物体边界: X[%.3f, %.3f], Y[%.3f, %.3f]", 
                object_left_, object_right_, object_bottom_, object_top_);
    debug_object_bounds();  // 添加详细的物体边界调试信息
    RCLCPP_INFO(get_logger(), "修正后的物体中心: (%.3f, %.3f)", object_center_.x(), object_center_.y());

    // 触摸头偏移量信息
    RCLCPP_INFO(get_logger(), "触摸头末端偏移量: (%.3f, %.3f, %.3f) m", 
                touch_tip_offset_.x(), touch_tip_offset_.y(), touch_tip_offset_.z());
    RCLCPP_INFO(get_logger(), "触摸头末端相对于末端执行器延伸: %.1f cm", touch_tip_offset_.z() * 100.0);

    // 初始化变量
    q_curr_.setZero();  q_des_.setZero();
    J_.setZero();      ee_pos_.setZero();
    R_t_.setIdentity(); contact_pt_.setZero();
    advance_pos_integral_.setZero();  // 初始化主轴推进积分项

    // 订阅: 关节状态, 雅可比, 触觉, 末端位姿
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&HybridForceTangentialController::on_js, this, _1));
    sub_jacobian_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/jacobian", 10, std::bind(&HybridForceTangentialController::on_jac, this, _1));
    sub_wrench_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/touch_tip/wrench", rclcpp::SensorDataQoS(), std::bind(&HybridForceTangentialController::on_wrench, this, _1));
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10, std::bind(&HybridForceTangentialController::on_pose, this, _1));

    // 发布: 期望关节位置
    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/joint_position_controller/commands", 10);
    
    // 发布: 接触点信息
    pub_contact_points_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/contact_points", 10);
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&HybridForceTangentialController::control_loop, this));

    RCLCPP_INFO(get_logger(), "HybridForceTangentialController 初始化完成");
  }

private:

  // 获取状态名称
  std::string getStateName(ScanState state) const {
    switch (state) {
      case ScanState::INIT_SCAN: return "INIT_SCAN";
      case ScanState::PAUSE_MEASURE: return "PAUSE_MEASURE";
      case ScanState::ALIGN_ORIENT: return "ALIGN_ORIENT";
      case ScanState::UNFREEZE: return "UNFREEZE";
      case ScanState::HORIZONTAL_SCAN: return "HORIZONTAL_SCAN";
      case ScanState::CONTINUE_HORIZONTAL_SCAN: return "CONTINUE_HORIZONTAL_SCAN";
      case ScanState::ADVANCE_ON_AXIS: return "ADVANCE_ON_AXIS";
      case ScanState::DONE: return "DONE";
      default: return "UNKNOWN";
    }
  }

  // 打印当前状态
  void printCurrentState() const {
    RCLCPP_INFO(get_logger(), "[状态机] 当前状态: %s", getStateName(state_).c_str());
  }

  // 限制各关节最大速度
  Eigen::Matrix<double,7,1> qdot_max_;

  // 新成员，用来记录每次横向扫描的起始位置
  double lat_start_{0.0};

  // 测力后要去的下一个状态（区别 INIT_SCAN、横向、推进三种场景）
  ScanState next_after_measure_;

  // bool horiz_first_iter_{false};    // 横向扫描第一帧

  Eigen::Vector3d f_est_saved_;  // 新增，用于 UNFREEZE 恢复

  // 物体边界
  double object_left_, object_right_, object_bottom_, object_top_;

  bool   recovering_contact_{false};  // 是否正处于下压恢复阶段

  // --- 横向扫描状态标志 ---
  bool horizontal_scan_completed_{false};   ///< 本条 X 截面横向扫描是否已完成
  bool horiz_first_iter_{true};             ///< 横向扫描首帧标志
  bool reached_left_{false};                ///< 已触达左边界？
  bool reached_right_{false};               ///< 已触达右边界？

  // 新增：主轴推进阶段的锁存目标
  Eigen::Vector3d advance_target_{Eigen::Vector3d::Zero()};
  bool advance_target_set_{false};

  double current_scan_line_x_; // 新增：用于存储当前扫线的固定X坐标

  void clampJointVel(Eigen::Matrix<double,7,1> &qdot) {
      for (int i = 0; i < 7; ++i) {
        if (qdot(i) > qdot_max_(i))      qdot(i) = qdot_max_(i);
        else if (qdot(i) < -qdot_max_(i)) qdot(i) = -qdot_max_(i);
      }
  }

  // --- 回调函数 ---
  void on_js(const sensor_msgs::msg::JointState::SharedPtr msg) {
    have_js_ = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
      if (it == joint_names_.end()) continue;
      size_t idx = std::distance(joint_names_.begin(), it);
      q_curr_(idx) = msg->position[i];
    }
    if (!initialized_) {
      q_des_ = q_curr_;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "首次初始化期望关节位置");
    }
  }

  void on_jac(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 确保维度 6×7
    if (msg->layout.dim.size()==2 && msg->layout.dim[0].size==6 && msg->layout.dim[1].size==7) {
      for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 7; ++j)
          J_(i,j) = msg->data[i*7 + j];
      have_jacobian_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "收到错误维度的雅可比: [%u, %u]", 
                  msg->layout.dim[0].size, msg->layout.dim[1].size);
    }
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    if (msg->header.frame_id != "world") return;
    f_ext6_.head<3>() = Eigen::Vector3d(
      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    f_ext6_.tail<3>() = Eigen::Vector3d(
      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    have_wrench_ = true;
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    ee_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond q(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
    R_t_ = q.normalized().toRotationMatrix();
    
    // 计算触摸头末端的实际位置
    touch_tip_pos_ = ee_pos_ + R_t_ * touch_tip_offset_;
    
    have_pose_ = true;
  }

  // --- 主循环 ---
  void control_loop() {
    // 打印状态以便调试
    RCLCPP_DEBUG(get_logger(), "[Loop %lu] state=%d js=%d jac=%d wrench=%d pose=%d init=%d",
                  loop_count_, static_cast<int>(state_), have_js_, have_jacobian_, have_wrench_, have_pose_, initialized_);
    if (!(have_js_ && have_jacobian_ && have_wrench_ && have_pose_ && initialized_)) return;

    // 添加状态转换调试信息
    if (loop_count_ % 100 == 0) {  // 每100帧输出一次状态
      RCLCPP_INFO(get_logger(), "当前状态: %d, 循环次数: %zu", static_cast<int>(state_), loop_count_);
      if (state_ == ScanState::HORIZONTAL_SCAN) {
        debug_ee_position();
      }
    }

    // 不再需要检查法向力，因为INIT_SCAN现在使用位置控制

    switch (state_) {

    case ScanState::INIT_SCAN: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
      
      // 1) 持续检测接触力
      double f_norm = f_ext6_.head<3>().norm();
      
      // 2) 检查是否已经接触成功（力大于25N）
      if (f_norm >= f_high_) {
        RCLCPP_INFO(get_logger(), "INIT_SCAN: 检测到接触力 %.3f N >= %.3f N，接触成功！", 
                    f_norm, f_high_);
        
        // 记录触摸头末端的接触点，而不是末端执行器位置
        contact_pt_ = touch_tip_pos_;
        RCLCPP_INFO(get_logger(), "接触成功, 触摸头末端接触点=(%.3f,%.3f,%.3f), 末端执行器=(%.3f,%.3f,%.3f)",
                    contact_pt_.x(), contact_pt_.y(), contact_pt_.z(),
                    ee_pos_.x(), ee_pos_.y(), ee_pos_.z());
        
        // 进入测力窗口
        next_after_measure_ = ScanState::ALIGN_ORIENT;
        state_ = ScanState::PAUSE_MEASURE;
        printCurrentState();
        pause_start_ = now();
        f_window_.clear();
        break;
      }
      
      // 3) 计算目标位置
      Eigen::Vector3d target_pos(init_target_x_, init_target_y_, init_target_z_);
      
      // 4) 计算位置误差
      Eigen::Vector3d pos_error = target_pos - ee_pos_;
      double pos_error_norm = pos_error.norm();
      
      // 5) 检查是否到达目标位置（即使到达目标位置，如果没有接触力也要继续）
      if (pos_error_norm <= init_position_tolerance_) {
        RCLCPP_INFO(get_logger(), "INIT_SCAN: 已到达目标位置 (%.3f, %.3f, %.3f), 误差=%.3f m，等待接触", 
                    target_pos.x(), target_pos.y(), target_pos.z(), pos_error_norm);
        
        // 到达目标位置但未接触，继续等待接触或微调位置
        // 可以在这里添加微调逻辑，比如轻微下压
        Eigen::Vector3d down_velocity(0, 0, -0.02); // 轻微下压速度，提高速度
        
        Eigen::Matrix<double,6,1> x_dot = Eigen::Matrix<double,6,1>::Zero();
        x_dot.head<3>() = down_velocity;
        
        Eigen::Matrix<double,6,6> W = J_ * J_.transpose() + lambda_ * lambda_ * Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_dot);
        
        clampJointVel(qdot);
        q_des_ += qdot * dt_;
        publish_cmd();
        
        break;
      }
      
      // 6) 位置控制：计算期望速度
      Eigen::Vector3d desired_velocity = pos_error.normalized() * init_position_speed_;
      
      // 7) 姿态控制：保持末端垂直于xy平面（Z轴向下）
      Eigen::Vector3d desired_z_axis(0, 0, -1); // Z轴向下，垂直于xy平面
      
      // 8) 计算姿态误差（使用叉积计算旋转轴和角度）
      Eigen::Vector3d current_z_axis = R_t_.col(2); // 当前工具Z轴
      Eigen::Vector3d rotation_axis = current_z_axis.cross(desired_z_axis);
      double rotation_angle = std::acos(std::clamp(current_z_axis.dot(desired_z_axis), -1.0, 1.0));
      
      // 计算角速度：ω = rotation_axis × rotation_angle × gain
      Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
      if (rotation_axis.norm() > 1e-6) {
        rotation_axis.normalize();
        angular_velocity = rotation_axis * rotation_angle * orientation_gain_;
      }
      
      // 9) 通过雅可比映射到关节速度（位置+姿态）
      Eigen::Matrix<double,6,1> x_dot = Eigen::Matrix<double,6,1>::Zero();
      x_dot.head<3>() = desired_velocity;  // 位置控制
      x_dot.tail<3>() = angular_velocity;  // 姿态控制：角速度
      
      Eigen::Matrix<double,6,6> W = J_ * J_.transpose() + lambda_ * lambda_ * Eigen::Matrix<double,6,6>::Identity();
      Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_dot);
      
      // 10) 关节速度限幅
      clampJointVel(qdot);
      
      // 11) 更新关节位置并发布命令
      q_des_ += qdot * dt_;
      publish_cmd();
      
      // 12) 输出调试信息
      if (loop_count_ % 100 == 0) {
        RCLCPP_INFO(get_logger(), "INIT_SCAN: 目标(%.3f,%.3f,%.3f) 触摸头末端(%.3f,%.3f,%.3f) 末端执行器(%.3f,%.3f,%.3f) 误差=%.3f m, 接触力=%.3f N", 
                    target_pos.x(), target_pos.y(), target_pos.z(),
                    touch_tip_pos_.x(), touch_tip_pos_.y(), touch_tip_pos_.z(),
                    ee_pos_.x(), ee_pos_.y(), ee_pos_.z(),
                    pos_error_norm, f_norm);
        
        // 输出姿态信息
        RCLCPP_INFO(get_logger(), "姿态控制: 当前Z轴[%.3f,%.3f,%.3f] 期望Z轴[%.3f,%.3f,%.3f] 角度误差=%.3f°", 
                    current_z_axis.x(), current_z_axis.y(), current_z_axis.z(),
                    desired_z_axis.x(), desired_z_axis.y(), desired_z_axis.z(),
                    rotation_angle * 180.0 / M_PI);
        
        if (rotation_axis.norm() > 1e-6) {
          RCLCPP_INFO(get_logger(), "旋转轴[%.3f,%.3f,%.3f] 角速度[%.3f,%.3f,%.3f]", 
                      rotation_axis.x(), rotation_axis.y(), rotation_axis.z(),
                      angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        }
      }
      
      break;
    }

    case ScanState::PAUSE_MEASURE: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
      /********** 0) 导纳执行（仅在未冻结时） **********/
      if (!admittance_frozen_) {
        compute_admittance_single();   // ★ 修复：使用单次导纳控制，避免过度运动
      }

      /********** 1) 首次进入：冻结导纳，开始静态测力 **********/
      if (!admittance_frozen_ && !recovering_contact_) {
        B_inv_saved_  = B_inv_;
        f_des_saved_  = f_des_;
        B_inv_.setZero();
        admittance_frozen_ = true;
        RCLCPP_INFO(get_logger(), "冻结导纳, 进入静态测力窗口");
      }

      /********** 2) 收集外力样本 **********/
      f_window_.push_back(f_ext6_);

      bool enough_time   = (now() - pause_start_).seconds() >= T_pause_;
      bool enough_sample = f_window_.size() >= static_cast<size_t>(N_min_);
      
      // 在恢复阶段，不等待样本收集，直接检查当前力
      if (recovering_contact_) {
        double f_mag = f_ext6_.head<3>().norm();
        
        // 每100帧输出一次恢复状态
        if (loop_count_ % 100 == 0) {
          RCLCPP_INFO(get_logger(), "恢复阶段: 当前力=%.3f N, 目标力=%.3f N", 
                      f_mag, desired_contact_force_);
        }
        
        if (f_mag >= f_low_) {
          recovering_contact_ = false;

          // 再次冻结导纳，进入正式静态测力
          B_inv_saved_  = B_inv_;
          B_inv_.setZero();
          admittance_frozen_ = true;

          f_window_.clear();
          pause_start_ = now();

          RCLCPP_INFO(get_logger(),
            "已重新接触 (|f|=%.3f N)，重新进入静态测力窗口", f_mag);
        }
        break;  // 恢复阶段直接退出，继续下压
      }
      
      if (!(enough_time && enough_sample)) break;   // 样本不足 → 继续等待

      /********** 3) 计算中位数外力 **********/
      auto  f_est   = median6(f_window_);
      Eigen::Vector3d f_vec = f_est.head<3>();
      double f_mag   = f_vec.norm();

      /********** 4) |f| 过小 → 失去接触，解除冻结让导纳下压 **********/
      if (f_mag < f_low_ || std::isnan(f_mag)) {

        if (admittance_frozen_) {
          B_inv_ = B_inv_saved_;           // 恢复导纳阻尼
          admittance_frozen_ = false;
          
          // 设置向下的期望力，让机器人下压重新接触
          if (n_hat_.norm() > 1e-6) {
            // 如果有法向量，沿法向量方向下压
            f_des_.head<3>() = -n_hat_ * desired_contact_force_;
          } else {
            // 否则沿Z轴负方向下压
            f_des_.head<3>() = Eigen::Vector3d(0, 0, -desired_contact_force_);
          }
          f_des_.tail<3>().setZero();  // 力矩设为0
        }

        recovering_contact_ = true;
        f_window_.clear();
        pause_start_ = now();              // 重新计时

        RCLCPP_WARN(get_logger(),
          "失去接触 (|f|=%.3f N)，解除冻结导纳并开始下压恢复", f_mag);
        break;                             // 仍停留在 PAUSE_MEASURE
      }

      /********** 6) 力有效且稳定 ——> 正常测力逻辑 **********/
      f_est_saved_ = f_vec;

      if (next_after_measure_ == ScanState::ALIGN_ORIENT) {
        n_hat_ = -f_vec.normalized();
        RCLCPP_INFO(get_logger(),
          "测力(ALIGN): f_est=[%.3f,%.3f,%.3f], n_hat=[%.3f,%.3f,%.3f]",
          f_est.x(), f_est.y(), f_est.z(),
          n_hat_.x(), n_hat_.y(), n_hat_.z());
        
        // 发布测力点
        publishContactPoint("ALIGN_ORIENT");
      } else if (next_after_measure_ == ScanState::CONTINUE_HORIZONTAL_SCAN) {
        // 继续横向扫描时，保持现有的法向量，不需要重新计算
        RCLCPP_INFO(get_logger(),
          "测力(CONTINUE): f_est=[%.3f,%.3f,%.3f]，保持 n_hat=[%.3f,%.3f,%.3f]",
          f_est.x(), f_est.y(), f_est.z(),
          n_hat_.x(), n_hat_.y(), n_hat_.z());
        
        // 发布测力点
        publishContactPoint("CONTINUE_HORIZONTAL_SCAN");
      } else {
        RCLCPP_INFO(get_logger(),
          "测力(HORIZ): f_est=[%.3f,%.3f,%.3f]，保持 n_hat=[%.3f,%.3f,%.3f]",
          f_est.x(), f_est.y(), f_est.z(),
          n_hat_.x(), n_hat_.y(), n_hat_.z());
        
        // 发布测力点
        publishContactPoint("HORIZONTAL_SCAN");
      }

      /********** 7) 跳转到测力前预定的下一状态 **********/
      state_ = next_after_measure_;
      printCurrentState();
      break;
    }



    case ScanState::ALIGN_ORIENT: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
    // --- 进入本阶段前须已执行：
    //     B_inv_.setZero();
    //     f_des_ = f_est_;

    // 1) 读取当前工具Z轴与力
    Eigen::Vector3d z_tcp = R_t_.col(2);            // 工具Z轴
    Eigen::Vector3d f_ext = f_ext6_.head<3>();      // 当前外力
    Eigen::Vector2d f_lat(f_ext.x(), f_ext.y());    // 切向力 (X,Y)
    double lat_norm = f_lat.norm();

    // 2) 计算姿态误差角
    double cos_ang = std::clamp(z_tcp.dot(n_hat_), -1.0, 1.0);
    double theta   = std::acos(cos_ang);

    // 3) 收敛判据：切向力 ≈ 0 且 角度 ≤ 容差
    if (loop_count_ % 10 == 0) {  // 每10帧输出一次，减少日志量
      RCLCPP_INFO(get_logger(),
        "ALIGN_ORIENT: lat=%.3f N, θ=%.3f°",
        lat_norm, theta * 180.0/M_PI);
    }
    // if (lat_norm <= f_eps_ && theta <= theta_tol_)
    if (theta <= theta_tol_) {
      RCLCPP_INFO(get_logger(),
        "ALIGN_ORIENT 完成：lat=%.3f N, θ=%.3f°",
        lat_norm, theta * 180.0/M_PI);
      // std::this_thread::sleep_for(std::chrono::seconds(1));
      state_ = ScanState::UNFREEZE;
      printCurrentState();
      break;
    }

    // 4) 计算纯角速度 ω = ω̂ * (δθ/Δt)
    Eigen::Vector3d omega_hat = z_tcp.cross(n_hat_);
    if (omega_hat.norm() > 1e-6) {
      omega_hat.normalize();
    }
    double delta_theta = k_theta_ * theta;  
    Eigen::Vector3d omega = omega_hat * (delta_theta / dt_);
    
    // 【新增】限制角速度幅值，防止过快旋转
    if (omega.norm() > max_angular_velocity_) {
      omega = omega.normalized() * max_angular_velocity_;
      RCLCPP_WARN(get_logger(), "姿态回正角速度过大，已限制到%.3f rad/s", max_angular_velocity_);
    }

    // 5) 计算"刚体纯旋"所需线速度 v = –ω × r
    Eigen::Vector3d r      = ee_pos_ - contact_pt_;
    Eigen::Vector3d v_full = -omega.cross(r);
    // 去除任何法向分量，只留下切向成分
    Eigen::Vector3d v_tan  = v_full - (v_full.dot(n_hat_)) * n_hat_;
    
    // 【关键修复】限制线速度幅值，防止姿态回正时的过大位移
    double max_linear_velocity = 0.02;  // 最大线速度2cm/s
    double original_v_norm = v_tan.norm();
    if (v_tan.norm() > max_linear_velocity) {
      v_tan = v_tan.normalized() * max_linear_velocity;
      RCLCPP_WARN(get_logger(), "姿态回正线速度过大(%.4f→%.4f m/s)，已限制", 
                  original_v_norm, max_linear_velocity);
    }
    
    // 【调试】每50帧输出一次速度信息
    if (loop_count_ % 50 == 0) {
      double r_norm = r.norm();
      RCLCPP_INFO(get_logger(), 
        "姿态回正: θ=%.2f°, |r|=%.3fm, |ω|=%.3f, |v_tan|=%.4f m/s", 
        theta*180.0/M_PI, r_norm, omega.norm(), v_tan.norm());
    }

    // 6) 构造任务空间速度 x_d = [v_tan; ω]
    Eigen::Matrix<double,6,1> x_d;
    x_d.head<3>() = v_tan;
    x_d.tail<3>() = omega;

    // 7) 雅可比伪逆映射到关节速度并限幅
    Eigen::Matrix<double,6,6> W = J_ * J_.transpose()
                                + lambda_ * lambda_ * Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_d);
    clampJointVel(qdot);                  // 用户定义的关节速度限幅函数

    // 8) 更新并发布关节命令
    q_des_ += qdot * dt_;
    publish_cmd();

    break;
  }

    case ScanState::UNFREEZE: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
    // 恢复导纳, 并设定恒定期望接触力
    f_des_.head<3>() = -n_hat_ * desired_contact_force_;
    B_inv_ = B_inv_saved_;
    admittance_frozen_ = false;
    RCLCPP_INFO(get_logger(),
      "解冻导纳, 恒定接触力 f_des=[%.3f, %.3f, %.3f] (大小=%.1fN)",
      f_des_.x(), f_des_.y(), f_des_.z(), desired_contact_force_);

    // 【修复】轻微的法向导纳推动，避免过度运动
    // 只执行1次导纳控制，避免过度的位移累积
    compute_admittance_single();

    // 初始化横向扫描状态
    //dir_lat_ = +1;                               // 先向"正"方向扫

    // 记录本条扫线的起始横向投影值，用于后面位移判断
    lat_start_ = ee_pos_.head<2>().dot(lateral_axis_);
    RCLCPP_INFO(get_logger(),
      "横向扫描起始投影 lat_start=%.3f",
      lat_start_);
    
    horiz_first_iter_       = true;                // 标记下一帧是第一帧横向扫描
    // 测力窗口结束后要继续 HORIZONTAL_SCAN
    next_after_measure_ = ScanState::HORIZONTAL_SCAN;
    state_ = ScanState::HORIZONTAL_SCAN;
    printCurrentState();
    break;
  }

    case ScanState::HORIZONTAL_SCAN: {
      // 每100帧打印一次状态
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }

      // —— 可选：安全检查（保持与你工程里一致；如不需要可删）——
      {
        double safety_tolerance = 0.02;  // 2cm
        Eigen::Vector2d rel_pos = get_ee_relative_position(); // 末端相对物体中心 xy
        double distance_from_center = rel_pos.norm();
        double max_distance = std::sqrt(object_width_ * object_width_ +
                                        object_height_ * object_height_) / 2.0
                              + safety_tolerance;
        if (distance_from_center > max_distance) {
          RCLCPP_WARN(get_logger(), "末端执行器超出物体安全边界，停止扫描");
          debug_ee_position();
          debug_object_bounds();
          state_ = ScanState::DONE;
          break;
        }
      }

      // —— 子帧 A：首帧处理 —— 
      if (horiz_first_iter_) {
        // 关键：进入新的一条扫线时，重置左右边界触发标记
        reached_left_  = false;
        reached_right_ = false;

        // 记录当前扫线的固定X坐标，用于后续约束
        current_scan_line_x_ = ee_pos_.x();
        RCLCPP_INFO(get_logger(), "开始新扫线，固定X坐标: %.6f", current_scan_line_x_);

        // 切向速度（仅Y方向，X方向为0）
        Eigen::Vector3d d_lat(0.0, lateral_axis_.y() * dir_lat_, 0.0);
        if (std::abs(d_lat.y()) > 1e-6) {
          d_lat.normalize();
        }

        // 切向导纳（仅Y方向）
        Eigen::Matrix<double,6,1> x_dot_t = Eigen::Matrix<double,6,1>::Zero();
        x_dot_t.head<3>() = lateral_speed_ * d_lat;
        // 强制X方向切向速度为0
        x_dot_t(0) = 0.0;

        // 法向导纳（保持接触力，但限制X方向）
        Eigen::Matrix<double,6,1> delta_f = f_ext6_ - f_des_saved_;
        Eigen::Matrix<double,6,1> x_dot_n = B_inv_ * delta_f;
        x_dot_n(0) = 0.0;  // 强制法向导纳不产生X方向速度

        // 超强X坐标约束控制：强制X坐标回到固定值
        double x_error = current_scan_line_x_ - ee_pos_.x();
        double x_gain = 100.0;  // 进一步增加X位置控制增益
        Eigen::Matrix<double,6,1> x_dot_x = Eigen::Matrix<double,6,1>::Zero();
        x_dot_x(0) = x_gain * x_error;  // X方向位置控制

        // 合成控制：X位置约束 + Y切向速度 + 法向导纳（X方向被约束）
        Eigen::Matrix<double,6,1> x_dot = x_dot_x + x_dot_t + x_dot_n;
        // 三重保险：确保X方向速度完全由约束控制
        x_dot(0) = x_gain * x_error;
        
        // 最终强制X方向速度为0（终极保险）
        x_dot(0) = x_gain * x_error;
        
        // 调用专门函数强制X方向速度为0
        force_x_velocity_zero(x_dot, x_gain, x_error);
        
        Eigen::Matrix<double,6,6> W     = J_ * J_.transpose()
                                     + lambda_ * lambda_ * Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_dot);
        clampJointVel(qdot);
        q_des_ += qdot * dt_;
        publish_cmd();

        // 标记下帧开始正常检测边界
        horiz_first_iter_ = false;
        RCLCPP_DEBUG(get_logger(), "首帧混合运动完成，边界标志已重置，X坐标固定为%.6f", current_scan_line_x_);
        break;
      }

      // —— 子帧 B：常规混合运动 + 位置边界检测 —— 
      {
        // 切向速度（仅Y方向，X方向为0）
        Eigen::Vector3d d_lat(0.0, lateral_axis_.y() * dir_lat_, 0.0);
        if (std::abs(d_lat.y()) > 1e-6) {
          d_lat.normalize();
        }

        // 切向导纳（仅Y方向）
        Eigen::Matrix<double,6,1> x_dot_t = Eigen::Matrix<double,6,1>::Zero();
        x_dot_t.head<3>() = lateral_speed_ * d_lat;
        // 强制X方向切向速度为0
        x_dot_t(0) = 0.0;

        // 法向导纳（保持接触力，但限制X方向）
        Eigen::Matrix<double,6,1> delta_f = f_ext6_ - f_des_saved_;
        Eigen::Matrix<double,6,1> x_dot_n = B_inv_ * delta_f;
        x_dot_n(0) = 0.0;  // 强制法向导纳不产生X方向速度

        // 超强X坐标约束控制：强制X坐标回到固定值
        double x_error = current_scan_line_x_ - ee_pos_.x();
        double x_gain = 100.0;  // 进一步增加X位置控制增益
        Eigen::Matrix<double,6,1> x_dot_x = Eigen::Matrix<double,6,1>::Zero();
        x_dot_x(0) = x_gain * x_error;  // X方向位置控制

        // 合成控制：X位置约束 + Y切向速度 + 法向导纳（X方向被约束）
        Eigen::Matrix<double,6,1> x_dot = x_dot_x + x_dot_t + x_dot_n;
        // 三重保险：确保X方向速度完全由约束控制
        x_dot(0) = x_gain * x_error;
        
        // 最终强制X方向速度为0（终极保险）
        x_dot(0) = x_gain * x_error;
        
        // 调用专门函数强制X方向速度为0
        force_x_velocity_zero(x_dot, x_gain, x_error);
        
        Eigen::Matrix<double,6,6> W     = J_ * J_.transpose()
                                     + lambda_ * lambda_ * Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_dot);
        clampJointVel(qdot);
        q_des_ += qdot * dt_;
        publish_cmd();

        // 计算横向投影（先取 2D 再点乘，避免维度不匹配）
        Eigen::Vector2d lat_axis2d = lateral_axis_.head<2>();
        if (lat_axis2d.norm() > 1e-12) lat_axis2d.normalize();
        const Eigen::Vector2d ee_xy = ee_pos_.head<2>();
        double curr_lat  = ee_xy.dot(lat_axis2d);
        double delta_lat = curr_lat - lat_start_;
        constexpr double kEdgeEps = 1e-3; // 离起点至少 1mm 再允许触发，避免"零位抖动"误触

        // ——★ 修复：双向边界检测，碰到任一边界即推进 —— 
        bool left_raw  = reached_object_boundary(-1);
        bool right_raw = reached_object_boundary(+1);

        // 只有离开起始点一定距离后才考虑触边
        bool hit_left  = (std::abs(delta_lat) > kEdgeEps) && left_raw;
        bool hit_right = (std::abs(delta_lat) > kEdgeEps) && right_raw;

        // 调试：打印边界检测详情
        if (loop_count_ % 20 == 0) {
          RCLCPP_INFO(get_logger(), "边界检测: curr_lat=%.4f, delta_lat=%.4f, left_raw=%d, right_raw=%d, hit_left=%d, hit_right=%d", 
                      curr_lat, delta_lat, left_raw, right_raw, hit_left, hit_right);
          
          // 打印物体边界用于调试
          Eigen::Vector2d corners[4] = {
            Eigen::Vector2d(object_left_, object_bottom_),
            Eigen::Vector2d(object_left_, object_top_),
            Eigen::Vector2d(object_right_, object_bottom_),
            Eigen::Vector2d(object_right_, object_top_)
          };
          
          double min_lat = std::numeric_limits<double>::max();
          double max_lat = std::numeric_limits<double>::lowest();
          
          for (const auto& corner : corners) {
            double lat = corner.dot(lat_axis2d);
            min_lat = std::min(min_lat, lat);
            max_lat = std::max(max_lat, lat);
          }
          
          RCLCPP_INFO(get_logger(), "物体横向边界: min=%.4f, max=%.4f, 容差=%.4f", min_lat, max_lat, 0.005);
        }

        if (hit_left || hit_right) {
          // 确定命中方向
          int dir_at_hit = hit_right ? +1 : -1;
          
          // 无论左右边界，只要碰到就推进，但重置边界状态防止重复
          RCLCPP_INFO(get_logger(),
                      "触到%s边界，进入主轴推进（curr_lat=%.4f, Δlat=%.4f）",
                      (dir_at_hit > 0 ? "右" : "左"), curr_lat, delta_lat);

          // 重置所有边界标志，为下一条扫线做准备
          reached_left_ = false;
          reached_right_ = false;
          
          // 为下一条扫线设置方向：如果碰到右边界，下一条向左扫；碰到左边界，下一条向右扫
          dir_lat_ = -dir_at_hit;

          // 将下一条扫线的起点设为当前投影
          lat_start_ = curr_lat;

          // 切到主轴推进
          state_ = ScanState::ADVANCE_ON_AXIS;
          printCurrentState();
          RCLCPP_INFO(get_logger(), "进入主轴推进状态");
          break;
        }

        // 调试信息：每50帧打印一次边界检测状态
        if (loop_count_ % 50 == 0) {
          RCLCPP_DEBUG(get_logger(),
                       "边界检测: left_raw=%d right_raw=%d, dir_lat=%d, Δlat=%.4f",
                       left_raw ? 1 : 0, right_raw ? 1 : 0, dir_lat_, delta_lat);
        }

        // 进度信息：每100帧打印一次
        if (loop_count_ % 100 == 0) {
          debug_ee_position();
          RCLCPP_INFO(get_logger(),
                      "横向进度: Δlat=%.3f/%.3f, dir_lat=%d, 目标步长=%.3f",
                      delta_lat, lateral_step_, dir_lat_, lateral_step_);
          
          // 显示X坐标约束效果
          double x_deviation = std::abs(ee_pos_.x() - current_scan_line_x_);
          RCLCPP_INFO(get_logger(),
                      "X坐标约束: 当前X=%.6f, 固定X=%.6f, 偏差=%.6f m",
                      ee_pos_.x(), current_scan_line_x_, x_deviation);
          
          // 如果X坐标偏差过大，发出警告
          if (x_deviation > 0.002) {  // 2mm阈值
            RCLCPP_WARN(get_logger(),
                        "X坐标偏差过大(%.6f m)，需要加强约束控制！", x_deviation);
          }
          
          // 显示扫描覆盖范围
          double scan_progress_x = (ee_pos_.x() - object_left_) / (object_right_ - object_left_) * 100.0;
          RCLCPP_INFO(get_logger(),
                      "主轴扫描进度: X=%.3f (%.1f%%), 范围[%.3f, %.3f]",
                      ee_pos_.x(), scan_progress_x, object_left_, object_right_);
          
          // 显示X方向速度约束状态
          RCLCPP_INFO(get_logger(),
                      "X方向速度约束: 切向速度X=0.0, 法向导纳X=0.0, 约束控制X=%.6f",
                      x_gain * x_error);
        }

        // 3) 横向步长完成检测 -> 进入测力暂停
        if (std::abs(delta_lat) >= lateral_step_) {
          next_after_measure_ = ScanState::CONTINUE_HORIZONTAL_SCAN;
          state_              = ScanState::PAUSE_MEASURE;
          printCurrentState();
          pause_start_        = now();
          f_window_.clear();
          RCLCPP_INFO(get_logger(),
                      "一步切向完成，进入测力对齐: Δlat=%.3f/%.3f", delta_lat, lateral_step_);
          
          // 记录横向扫描点
          publishContactPoint("HORIZONTAL_SCAN_STEP");
        } else {
          RCLCPP_DEBUG(get_logger(),
                       "继续横向移动: Δlat=%.3f/%.3f", delta_lat, lateral_step_);
        }

        break;
      }
    }



    // 新增状态：继续横向扫描（跳过对齐和解冻）
    case ScanState::CONTINUE_HORIZONTAL_SCAN: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
      
      // 确保导纳控制是激活的
      if (admittance_frozen_) {
        admittance_frozen_ = false;
        B_inv_ = B_inv_saved_;
        RCLCPP_INFO(get_logger(), "CONTINUE_HORIZONTAL_SCAN: 重新激活导纳控制");
      }
      
      // 直接继续横向扫描，不需要重新对齐和解冻
      // 重新记录起始位置
      lat_start_ = ee_pos_.head<2>().dot(lateral_axis_);
      horiz_first_iter_ = true;  // 重新开始首帧逻辑
      reached_left_ = false;     // 重置左边界标志
      reached_right_ = false;    // 重置右边界标志
      
      RCLCPP_INFO(get_logger(), "继续横向扫描，新起始位置: lat_start=%.3f", lat_start_);
      
      // 直接进入横向扫描
      state_ = ScanState::HORIZONTAL_SCAN;
      printCurrentState();
      break;
    }

    case ScanState::ADVANCE_ON_AXIS: {
      if (loop_count_ % 100 == 0) {
        printCurrentState();
      }
      
      //sleep(1);
      // std::this_thread::sleep_for(std::chrono::seconds(5));

      /* 1) 首次进入时：冻结导纳 + 锁存目标 ------------------------------- */
      if (!admittance_frozen_) {
        B_inv_.setZero();          // 冻结导纳矩阵
        f_des_.setZero();          // 清期望力
        admittance_frozen_ = true;

        // ——只在首次进入时"锁存"目标点：当前 EE 位姿 + 步长（沿 X 正方向）——
        advance_target_      = ee_pos_;
        advance_target_.x() += step_length_;   // 主轴方向为世界系 X，如需换轴，这里改
        // 如需保持当前 Y/Z，不必额外改，因为 advance_target_ 已取自 ee_pos_
        advance_target_set_  = true;

        RCLCPP_INFO(get_logger(),
          "ADVANCE阶段：冻结导纳并锁存目标 (%.3f, %.3f, %.3f)",
          advance_target_.x(), advance_target_.y(), advance_target_.z());
      }

      /* 2) 是否已到达主轴终止边界（例如工件最右侧） ----------------------- */
      if (ee_pos_.x() >= object_right_ - 0.001) {  // 1mm 容差，确保完整扫描
        RCLCPP_INFO(get_logger(), "已到达物体主轴边界 (%.3f >= %.3f)，扫描完成", 
                    ee_pos_.x(), object_right_);
        state_ = ScanState::DONE;
        printCurrentState();
        // 退出前清理锁存标志
        advance_target_set_ = false;
        admittance_frozen_  = false;
        B_inv_              = B_inv_saved_;
        break;
      }

      /* 3) 纯位置控制：目标始终用"锁存"的 advance_target_ ----------------- */
      if (!advance_target_set_) {
        // 理论上不会发生；加保护以免目标意外丢失
        advance_target_      = ee_pos_;
        advance_target_.x() += step_length_;
        advance_target_set_  = true;
        RCLCPP_WARN(get_logger(), "advance_target_ 丢失，已按当前EE重建目标");
      }

      Eigen::Vector3d pos_error = advance_target_ - ee_pos_;

      // 仅沿主轴X方向推进，更直接、更稳定
      double ex = advance_target_.x() - ee_pos_.x();
      if (loop_count_ % 100 == 0) {
        RCLCPP_INFO(get_logger(),
          "ADVANCE轴向推进: 目标X=%.3f, 当前X=%.3f, ex=%.4f, step=%.3f",
          advance_target_.x(), ee_pos_.x(), ex, step_length_);
      }

      /* 4) 追踪到锁存目标（只看X轴误差） ----------------------------------- */
      double tol = position_tolerance_;   // 使用参数化的位置控制容差
      if (std::abs(ex) > tol) {
        // 改进的一维PI控制，确保推进距离完全一致
        double kp = 8.0;  // 增加P增益，提高响应速度
        double ki = 1.0;  // 增加I增益，消除稳态误差
        
        // 积分项累积
        advance_ex_integral_ += ex * dt_;
        
        // 防积分饱和（按最大步长的一半）
        double max_i = step_length_ * 0.5;
        if (std::abs(advance_ex_integral_) > max_i) {
          advance_ex_integral_ = (advance_ex_integral_ > 0 ? max_i : -max_i);
        }

        // 计算控制输出
        double vx = kp * ex + ki * advance_ex_integral_;
        
        // 限速：确保推进速度适中，既不太快也不太慢
        double max_speed = axis_speed_;
        if (std::abs(vx) > max_speed) {
          vx = (vx > 0 ? max_speed : -max_speed);
        }

        // 构建控制向量：仅沿X轴推进
        Eigen::Matrix<double,6,1> x_dot = Eigen::Matrix<double,6,1>::Zero();
        x_dot(0) = vx;  // 仅沿X轴

        // 雅可比伪逆映射到关节空间
        Eigen::Matrix<double,6,6> W = J_*J_.transpose()
                                    + lambda_*lambda_*Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,7,1> qdot = J_.transpose() * (W.inverse() * x_dot);

        // 关节速度限幅和更新
        clampJointVel(qdot);
        q_des_ += qdot * dt_;
        publish_cmd();
        
        // 每50帧输出推进状态
        if (loop_count_ % 50 == 0) {
          RCLCPP_INFO(get_logger(),
            "推进中: 目标=%.6f, 当前=%.6f, 误差=%.6f, 速度=%.6f, 积分=%.6f",
            advance_target_.x(), ee_pos_.x(), ex, vx, advance_ex_integral_);
        }
        
        break;   // 尚未到位，先退出 switch
      }
      
      // 到达目标后，记录实际推进距离并重置积分项
      double actual_advance = ee_pos_.x() - (advance_target_.x() - step_length_);
      RCLCPP_INFO(get_logger(),
        "推进完成: 目标步长=%.6f, 实际推进=%.6f, 误差=%.6f mm",
        step_length_, actual_advance, (step_length_ - actual_advance) * 1000.0);
      
      // 重置积分项，为下次推进做准备
      advance_pos_integral_.setZero();
      advance_ex_integral_ = 0.0;

      /* 5) 已到位 ——> 复位并进入测力窗口 -------------------------------- */
      // 目标已到，清除锁存标志；恢复导纳
      advance_target_set_ = false;
      admittance_frozen_  = false;
      B_inv_              = B_inv_saved_;

      // 可将 contact_pt_ 更新为"这次推进完成后的新基准"
      contact_pt_ = touch_tip_pos_;  // 使用触摸头末端位置
      RCLCPP_INFO(get_logger(),
        "主轴推进完成，触摸头末端新接触点=(%.3f, %.3f, %.3f)，末端执行器=(%.3f, %.3f, %.3f)，重新启用导纳控制",
        contact_pt_.x(), contact_pt_.y(), contact_pt_.z(),
        ee_pos_.x(), ee_pos_.y(), ee_pos_.z());

      // 复位横向扫描相关标志
      horizontal_scan_completed_ = false;
      horiz_first_iter_          = true;
      reached_left_              = false;
      reached_right_             = false;

      // 进入"测力 → 姿态回正"流程
      next_after_measure_ = ScanState::ALIGN_ORIENT;
      state_              = ScanState::PAUSE_MEASURE;
      printCurrentState();
      pause_start_        = now();
      f_window_.clear();

      break;
    }




    case ScanState::DONE:
      // 7 DONE: 扫描完成
      printCurrentState();
      RCLCPP_INFO(get_logger(), "扫描完成, 总循环次数=%zu", loop_count_);
      break;
    }

    loop_count_++;
  }

  // 力导纳计算函数
  void compute_admittance() {
    // 运行三次导纳控制以获得更好的响应
    for (int i = 0; i < 5; ++i) {
      Eigen::Matrix<double,6,1> delta = f_ext6_ - f_des_;
      Eigen::Matrix<double,6,1> xdn   = B_inv_ * delta;
      Eigen::Matrix<double,6,6> W     = J_*J_.transpose() + lambda_*lambda_*Eigen::Matrix<double,6,6>::Identity();
      Eigen::Matrix<double,6,1> xdot  = xdn;
      Eigen::Matrix<double,6,6> Winv  = W.inverse();
      Eigen::Matrix<double,7,1> qdot  = J_.transpose() * (Winv * xdot);
      q_des_ += qdot * dt_;
    }
    publish_cmd();
  }

  // 单次导纳计算函数（用于避免过度运动）
  void compute_admittance_single() {
    // 只执行一次导纳控制，避免位移累积
    Eigen::Matrix<double,6,1> delta = f_ext6_ - f_des_;
    Eigen::Matrix<double,6,1> xdn   = B_inv_ * delta;
    Eigen::Matrix<double,6,6> W     = J_*J_.transpose() + lambda_*lambda_*Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,1> xdot  = xdn;
    Eigen::Matrix<double,6,6> Winv  = W.inverse();
    Eigen::Matrix<double,7,1> qdot  = J_.transpose() * (Winv * xdot);
    
    // 添加安全检查：限制单次位移量
    double max_single_displacement = 0.001; // 最大单次位移1mm
    for (int i = 0; i < 7; ++i) {
      double displacement = qdot(i) * dt_;
      if (std::abs(displacement) > max_single_displacement) {
        qdot(i) = (displacement > 0 ? max_single_displacement : -max_single_displacement) / dt_;
      }
    }
    
    q_des_ += qdot * dt_;
    publish_cmd();
    
    RCLCPP_INFO(get_logger(), "单次导纳控制完成，位移限制在%.1fmm内", max_single_displacement * 1000.0);
  }

  // 发布关节命令
  void publish_cmd() {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(q_des_.data(), q_des_.data()+7);
    
    // 添加调试信息
    static int debug_counter = 0;
    if (debug_counter++ % 100 == 0) {  // 每100次循环输出一次
      RCLCPP_INFO(get_logger(), "发布关节命令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  q_des_(0), q_des_(1), q_des_(2), q_des_(3), q_des_(4), q_des_(5), q_des_(6));
    }
    
    pub_cmd_->publish(cmd);
  }

  // 检查是否到达物体边界
  bool reached_object_boundary(int lateral_direction) const {
    // 添加容差，避免在边界附近反复触发
    double boundary_tolerance = 0.003; // 3mm的容差，减少误触发
    
    // 计算当前末端执行器在横向轴上的投影
    Eigen::Vector2d lat_axis2d = lateral_axis_.head<2>();
    if (lat_axis2d.norm() > 1e-12) lat_axis2d.normalize();
    Eigen::Vector2d ee_pos2d = ee_pos_.head<2>();
    double lat_projection = lat_axis2d.dot(ee_pos2d - object_center_.head<2>());
    
    // 检查是否到达左边界或右边界
    bool at_left_boundary = lat_projection <= -object_height_ / 2.0 + boundary_tolerance;
    bool at_right_boundary = lat_projection >= object_height_ / 2.0 - boundary_tolerance;
    
    // 无论移动方向如何，只要到达任一边界就返回true
    if (lateral_direction > 0) {  // 向右移动
      return at_right_boundary;
    } else {  // 向左移动
      return at_left_boundary;
    }
  }

  // 强制X方向速度为0的函数
  void force_x_velocity_zero(Eigen::Matrix<double,6,1>& x_dot, double x_gain, double x_error) {
    // 强制X方向速度完全为0，只保留约束控制
    x_dot(0) = x_gain * x_error;
    
    // 确保其他方向的速度不受影响
    // x_dot(1) 和 x_dot(2) 保持Y和Z方向的速度
    // x_dot(3), x_dot(4), x_dot(5) 保持角速度
  }

  // 获取当前末端执行器相对于物体中心的位置
  Eigen::Vector2d get_ee_relative_position() const {
    return ee_pos_.head<2>() - object_center_;
  }

  // 调试函数：显示末端执行器位置状态
  void debug_ee_position() const {
    Eigen::Vector2d rel_pos = get_ee_relative_position();
    RCLCPP_INFO(get_logger(), 
      "EE位置: (%.3f, %.3f), 相对中心: (%.3f, %.3f)",
      ee_pos_.x(), ee_pos_.y(), 
      rel_pos.x(), rel_pos.y());
  }

  // 调试函数：显示物体边界设置
  void debug_object_bounds() const {
    RCLCPP_INFO(get_logger(), 
      "物体边界设置: 中心(%.3f, %.3f), 尺寸(%.3f x %.3f), 边界X[%.3f, %.3f], Y[%.3f, %.3f]",
      object_center_.x(), object_center_.y(),
      object_width_, object_height_,
      object_left_, object_right_, object_bottom_, object_top_);
  }

  // 发布接触点信息
  void publishContactPoint(const std::string& state_name) {
    geometry_msgs::msg::PoseStamped contact_msg;
    contact_msg.header.stamp = this->now();
    contact_msg.header.frame_id = "world";
    
    // 使用触摸头末端的实际位置，而不是末端执行器位置
    contact_msg.pose.position.x = touch_tip_pos_.x();
    contact_msg.pose.position.y = touch_tip_pos_.y();
    contact_msg.pose.position.z = touch_tip_pos_.z();
    
    // 将法向量转换为四元数
    Eigen::Vector3d z_axis = n_hat_;
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
    if (std::abs(z_axis.dot(x_axis)) > 0.9) {
      x_axis = Eigen::Vector3d::UnitY();
    }
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();
    
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    
    Eigen::Quaterniond q(rotation_matrix);
    contact_msg.pose.orientation.w = q.w();
    contact_msg.pose.orientation.x = q.x();
    contact_msg.pose.orientation.y = q.y();
    contact_msg.pose.orientation.z = q.z();
    
    pub_contact_points_->publish(contact_msg);
    
    RCLCPP_INFO(get_logger(), "发布测力点 [%s]: 触摸头末端(%.3f, %.3f, %.3f), 末端执行器(%.3f, %.3f, %.3f), 法向量: (%.3f, %.3f, %.3f)", 
                state_name.c_str(), 
                touch_tip_pos_.x(), touch_tip_pos_.y(), touch_tip_pos_.z(),
                ee_pos_.x(), ee_pos_.y(), ee_pos_.z(),
                n_hat_.x(), n_hat_.y(), n_hat_.z());
  }

  // 6D 力中位数滤波
  Eigen::Matrix<double,6,1> median6(const std::deque<Eigen::Matrix<double,6,1>>& w) const {
    Eigen::Matrix<double,6,1> m;
    for (int i=0; i<6; ++i) {
      std::vector<double> v; v.reserve(w.size());
      for (auto &f: w) v.push_back(f(i));
      size_t k = v.size()/2;
      std::nth_element(v.begin(), v.begin()+k, v.end());
      m(i) = v[k];
    }
    return m;
  }

  // 成员变量
  ScanState state_;

  // 初始化目标位置已通过参数设置，不再需要动态计算方向

  bool initialized_{false}, have_js_{false}, have_jacobian_{false}, have_wrench_{false}, have_pose_{false};
  bool admittance_frozen_;
  double f_high_, f_low_, step_length_, lateral_step_;
  double axis_speed_, lateral_speed_, T_pause_, f_eps_, theta_tol_, k_theta_;
  double max_angular_velocity_;  // 最大角速度限制
  double init_target_x_, init_target_y_, init_target_z_;  // 初始化目标位置
  double init_position_tolerance_;  // 初始化位置容差
  double init_position_speed_;  // 初始化位置控制速度
  double orientation_gain_;  // 姿态控制增益
  double position_tolerance_;  // 位置控制容差
  double object_width_, object_height_;  // 物体尺寸
  Eigen::Vector3d advance_pos_integral_;  // 主轴推进位置控制积分项
  double advance_ex_integral_{0.0};       // 主轴X轴误差积分项（标量）
  int N_min_;
  double desired_contact_force_;  // 期望恒定接触力大小
  const double dt_{0.01}, lambda_{0.1};
  Eigen::DiagonalMatrix<double,6> B_inv_, B_inv_saved_;
  Eigen::Matrix<double,6,1> f_des_, f_des_saved_, f_ext6_;
  Eigen::Matrix<double,6,7> J_;
  Eigen::Matrix<double,7,1> q_curr_, q_des_;
  Eigen::Vector3d ee_pos_, contact_pt_, n_hat_;
  Eigen::Matrix3d R_t_;
  Eigen::Vector2d main_axis_, lateral_axis_, object_center_;
  std::string ee_pose_topic_;
  std::deque<Eigen::Matrix<double,6,1>> f_window_;
  rclcpp::Time pause_start_;
  //bool bounds_found_[2];
  int dir_lat_ = 1;
  size_t loop_count_;
  std::vector<std::string> joint_names_{"fr3_joint1","fr3_joint2","fr3_joint3","fr3_joint4","fr3_joint5","fr3_joint6","fr3_joint7"};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    sub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_jacobian_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pub_contact_points_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  // 新增：触摸头末端偏移量（相对于末端执行器）
  Eigen::Vector3d touch_tip_offset_{0.0, 0.0, 0.257}; // 触摸头末端偏移：0.107 + 0.15 = 0.257m

  // 新增：触摸头末端实际位置
  Eigen::Vector3d touch_tip_pos_{Eigen::Vector3d::Zero()};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HybridForceTangentialController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
