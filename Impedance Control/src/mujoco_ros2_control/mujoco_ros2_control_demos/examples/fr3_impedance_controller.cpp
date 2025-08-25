// mujoco_ros2_control_demos/examples/fr3_impedance_controller.cpp
// mujoco_ros2_control_demos/examples/fr3_impedance_controller.cpp
#include <Eigen/Dense>  // Eigen 用于矩阵运算
#include <rclcpp/rclcpp.hpp>
#include <mujoco_ros2_control_demos/msg/joint_state_array.hpp> // 订阅的自定义关节状态消息
#include <std_msgs/msg/float64_multi_array.hpp>               // 发布力矩命令的消息类型
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <Eigen/Core>          
#include <Eigen/Geometry>      // 四元数需要

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
// 新增头文件用于KDL到Eigen以及ROS消息之间的转换
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 用于KDL::Rotation和Eigen::Quaterniond转换
#include <tf2_eigen/tf2_eigen.hpp> // 用于KDL::Frame到Eigen::Isometry3d转换

#include <geometry_msgs/msg/point_stamped.hpp> // 用于发布位置数据
#include <std_msgs/msg/float64_multi_array.hpp> 

class FR3ImpedanceController : public rclcpp::Node
{
public:
    FR3ImpedanceController() : Node("fr3_impedance_controller")
    {
        // 1. 创建订阅器，订阅状态管理节点发布的整合后的关节状态
        joint_state_subscription_ = this->create_subscription<mujoco_ros2_control_demos::msg::JointStateArray>(
            "/robot_state_for_controller",
            rclcpp::SensorDataQoS(), // 使用 SensorDataQoS 以获得最新的数据
            std::bind(&FR3ImpedanceController::jointStateCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Subscribing to /robot_state_for_controller...");

        // 2. 创建发布器，发布关节力矩命令
        effort_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands",
            10 // QoS history depth
        );
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Publishing to /effort_controller/commands...");

        // 初始化雅可比订阅器
        jacobian_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jacobian", // 订阅雅可比发布器发布的话题
            rclcpp::SensorDataQoS(), // 使用 SensorDataQoS 以确保获取最新数据
            std::bind(&FR3ImpedanceController::jacobianCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Subscribing to /jacobian...");


        // (可选) 关节轨迹发布器，纯阻抗控制通常不需要，但如果需要发送轨迹点给其他控制器则保留
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory",
            10
        );
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Publishing to /joint_trajectory_controller/joint_trajectory...");


        /************************************ publish position *****************************************************/
        current_position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/fr3_controller/current_position",
            10
        );
        desired_position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/fr3_controller/desired_position",
            10
        );
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Publishing current_position and desired_position.");

        published_torques_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/fr3_controller/published_torques", 
            10
        );

        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Publishing calculated torques to /fr3_controller/published_torques.");
        /************************************ publish position *****************************************************/


        // 初始化关节名称列表 (与 StateManagerNode 中保持一致)
        joint_names_ = {
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7"
        };
        num_joints_ = joint_names_.size();
        RCLCPP_INFO(this->get_logger(), "FR3ImpedanceController: Initialized for %zu joints.", num_joints_);



        // ======== 新增：重力补偿相关的初始化 ========
        tau_gravity_compensation_.setZero(num_joints_); // 初始化为零向量

        // ======== 新增：末端位姿相关的初始化 ========
        ee_position_ = Eigen::Vector3d::Zero(); // 初始化为零向量
        ee_orientation_ = Eigen::Quaterniond::Identity(); // 初始化为单位四元数

        // ======== 新增：传感器数据相关的初始化 ========
        sensor_force_ = Eigen::Vector3d::Zero(); // 初始化为零向量
        sensor_torque_ = Eigen::Vector3d::Zero(); // 初始化为零向量

        // 订阅重力补偿话题
        gravity_forces_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gravity_compensation", // 发布者发布的话题名
            rclcpp::SensorDataQoS(), // 使用传感器数据QoS以获得最新数据
            std::bind(&FR3ImpedanceController::gravityCompensationCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribing to /gravity_compensation for gravity compensation torques.");
        // ============================================

        // ======== 新增：末端位姿订阅器 ========
        ee_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ee_pose", // 订阅末端位姿话题
            rclcpp::SensorDataQoS(), // 使用传感器数据QoS以获得最新数据
            std::bind(&FR3ImpedanceController::eePoseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to /ee_pose for end-effector pose.");
        // ============================================

        // ======== 新增：传感器数据订阅器 ========
        sensor_wrench_subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/sensor_wrench", // 订阅传感器力/力矩数据话题
            rclcpp::SensorDataQoS(), // 使用传感器数据QoS以获得最新数据
            std::bind(&FR3ImpedanceController::sensorWrenchCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to /sensor_wrench for sensor force/torque data.");
        // ============================================




        // --- 轨迹规划相关初始化 ---
        // 这是轨迹的最终目标位置
        final_target_position_ << 0.3, -0.3, 0.55; // 期望 X, Y, Z (米) (矩形:0.15/0.2,-0.25,0.6)
        // position_d_ 将在 approachObject 中被轨迹规划器更新
        position_d_ = Eigen::Vector3d::Zero(); // 初始化为零，将由轨迹更新
        orientation_d_ = Eigen::Quaterniond::Identity();  // // 无旋转（单位四元数）
        
        total_motion_time_ = 30.0; // 整个轨迹的持续时间 (秒), 初始值为接近物体的轨迹时间
        // -------------------------------------------------------------------

        // 初始化零空间期望关节构型**
        // 设置为更保守的构型，避免关节限位
        q_d_nullspace_ << 0.0, -M_PI/6.0, 0.0, -M_PI/3.0, 0.0, M_PI/3.0, M_PI/6.0; 

        // **修改点3：初始化阻抗参数为6x6矩阵**
        cartesian_stiffness_.setIdentity(); // 初始化为单位矩阵
        // 设置位置刚度 (N/m) - 降低刚度以获得更好的稳定性
        cartesian_stiffness_.topLeftCorner(3,3) << 1500.0, 0.0, 0.0,
                                                    0.0, 1500.0, 0.0,
                                                    0.0, 0.0, 750.0;
        // 设置姿态刚度 (Nm/rad) - 降低刚度以获得更好的稳定性
        cartesian_stiffness_.bottomRightCorner(3,3) << 400.0, 0.0, 0.0,
                                                      0.0, 400.0, 0.0,
                                                      0.0, 0.0, 200.0; 
        
        cartesian_damping_.setIdentity(); // 初始化为单位矩阵
        // 阻尼比设为1 (临界阻尼)，D = 2 * sqrt(K)
        cartesian_damping_.topLeftCorner(3,3) =  2 * cartesian_stiffness_.topLeftCorner(3,3).cwiseSqrt(); // 位置阻尼
        cartesian_damping_.bottomRightCorner(3,3) =  2 * cartesian_stiffness_.bottomRightCorner(3,3).cwiseSqrt(); // 姿态阻尼
        
        //初始化零空间刚度**
        nullspace_stiffness_ = 5.0; // Nm/rad，零空间任务的刚度 - 降低以获得更好的稳定性

        // 初始化上一周期发送的力矩，用于力矩变化率限制
        tau_J_d_.setZero(num_joints_);

        
        //sleep(5);
    }

private:

    // 状态定义
              enum class ControllerState {
        KDL_INIT,
        STABLE_INIT,       // 初始稳定状态
        APPROACH_OBJECT,    // 靠近目标状态
        CONTACT_CONFIRM,    // 接触确认状态
        FORCE_TUNE,         // 力调整状态
        TANGENTIAL_SCAN,    // 切向扫描状态
        RETOUCH,            // 重新接触状态
        SCANNING_COMPLETED, // 扫描完成状态
    };


    // 将 ControllerState 转换为字符串
    std::string stateToString(ControllerState state) 
    {
        switch (state) {
            case ControllerState::KDL_INIT:          return "KDL_INIT";
            case ControllerState::STABLE_INIT:       return "STABLE_INIT";
            case ControllerState::APPROACH_OBJECT:   return "APPROACH_OBJECT";
            case ControllerState::CONTACT_CONFIRM:   return "CONTACT_CONFIRM";
            case ControllerState::FORCE_TUNE:        return "FORCE_TUNE";
            case ControllerState::TANGENTIAL_SCAN:   return "TANGENTIAL_SCAN";
            case ControllerState::RETOUCH:           return "RETOUCH";
            case ControllerState::SCANNING_COMPLETED:return "SCANNING_COMPLETED";
            default: return "UNKNOWN_STATE";
        }
    }

    // 状态机相关成员变量
    ControllerState current_state_ = ControllerState::KDL_INIT;
    
    // 力控制相关参数
    double target_force_ = 50.0;          // 目标接触力 (N)
    double force_tolerance_ = 20.0;        // 允许波动范围 (±N)
    double force_stable_time_ = 0.2;       // 力稳定持续时间 (s)
    double force_deadzone_ = 10.0;          // 力控制死区 (±N)
    rclcpp::Time force_stable_start_time_; // 力稳定开始时间
    bool is_force_stable_ = false;        // 力是否稳定标志
    
    // 传感器滤波参数
    double sensor_filter_alpha_ = 0.05;     // 传感器低通滤波系数
    bool sensor_filter_initialized_ = false; // 传感器滤波初始化标志
    double filtered_sensor_force_ = 0.0;   // 滤波后的传感器力值
    
    // 离散扫描参数
    double tangential_step_ = 0.05;        // 切向运动步长 (1cm)
    double retouch_step_ = 0.005;          // 重新接触步长 (5mm)
    double contact_force_threshold_ = 5.0; // 接触力阈值 (1N)
    
    // 扫描状态变量
    Eigen::Vector3d scan_start_position_; // 扫描起始位置
    Eigen::Vector3d current_scan_position_; // 当前扫描位置
    double tangential_scan_time_ = 0.2;   // 切向扫描时间 0.2 (s)
    double retouch_time_ = 1.0;           // 重新接触时间 (s)
    
    // 扫描计时相关变量
    rclcpp::Time scanning_start_time_;    // 扫描开始时间
    rclcpp::Time scanning_end_time_;      // 扫描结束时间
    bool scanning_timer_started_ = false; // 扫描计时器是否已启动
    bool scanning_timer_stopped_ = false; // 扫描计时器是否已停止
    
    // S形扫描参数
    struct ScanningParams {
        double x_min = 0.3; // 扫描区域X最小值 0.2
        double x_max = 0.55; // 扫描区域X最大值
        double y_min = -0.3; // 扫描区域Y最小值 -0.25
        double y_max = 0.15; // 扫描区域Y最大值
        double y_step = 0.025; // Y方向步长
        double z_height = 0.7; // 预设扫描高度
        double position_tolerance = 0.002; // 位置到达容差 0.002(m)
    } scan_params_;
    
    // S形扫描状态
    struct ScanningState {
        int current_line = 0;              // 当前扫描线索引
        int total_lines = 16;              // 总扫描线数量 15
        bool is_forward_direction = true;  // 当前是否为正向扫描
        double current_y = -0.25;          // 当前Y坐标
        bool is_scanning_complete = false; // 扫描是否完成
        bool is_at_boundary = false;       // 是否到达边界
        bool is_turning = false;           // 是否正在转弯（Y方向移动）
        bool x_position_locked = false;    // X位置是否已锁定
        double locked_x_position = 0.0;    // 锁定的X位置
        double target_y_position = 0.0;    // 目标Y位置
    } scan_state_;

    // 点云记录相关
    struct PointCloudData {
        std::vector<Eigen::Vector3d> positions;     // 位置点
        std::vector<Eigen::Vector3d> forces;        // 力数据
        std::vector<Eigen::Vector3d> torques;       // 力矩数据
        std::vector<double> timestamps;             // 时间戳
        std::vector<int> line_indices;              // 扫描线索引
        std::string filename;                       // 保存文件名
    } point_cloud_data_;

    // **S形扫描辅助函数**
    double getCurrentY(int line_index) {
        return scan_params_.y_min + line_index * scan_params_.y_step;
    }
    
    bool isForwardDirection(int line_index) {
        return (line_index % 2 == 0);  // 偶数行正向，奇数行反向
    }
    
    std::pair<double, double> getXRange(int line_index) {
        if (isForwardDirection(line_index)) {
            return {scan_params_.x_min, scan_params_.x_max};  // 0.15 → 0.55
        } else {
            return {scan_params_.x_max, scan_params_.x_min};  // 0.55 → 0.15
        }
    }
    
    bool isCurrentLineComplete() {
        if (!ee_pose_received_) return false;
        
        double current_x = ee_position_.x();
        double target_x = isForwardDirection(scan_state_.current_line) ? 
                         scan_params_.x_max : scan_params_.x_min;
        return std::abs(current_x - target_x) <= scan_params_.position_tolerance;
    }
    
    bool isYMovementComplete() {
        if (!ee_pose_received_) return false;
        
        double current_y = ee_position_.y();
        double target_y = getCurrentY(scan_state_.current_line);
        return std::abs(current_y - target_y) <= scan_params_.position_tolerance;
    }
    
    // 检查是否到达扫描边界
    bool isAtScanBoundary() 
    {
        if (!ee_pose_received_) return false;

        double current_x = ee_position_.x();
        double target_x = isForwardDirection(scan_state_.current_line) ? 
                         scan_params_.x_max : scan_params_.x_min;
        return std::abs(current_x - target_x) <= scan_params_.position_tolerance;
    }
    

    
    // 检查是否离开表面（接触力小于阈值）
    bool isOffSurface() {
        if (!sensor_wrench_received_) return false;
        return std::abs(sensor_force_.z()) < contact_force_threshold_;
    }
    
    // 传感器力值低通滤波（仅用于力控制阶段）
    double filterSensorForce(double raw_force) 
    {
        if (!sensor_filter_initialized_) 
        {
            filtered_sensor_force_ = raw_force;
            sensor_filter_initialized_ = true;
        } 
        else 
        {
            filtered_sensor_force_ = sensor_filter_alpha_ * raw_force + (1.0 - sensor_filter_alpha_) * filtered_sensor_force_;
        }
        return filtered_sensor_force_;
    }
    
    // 获取当前传感器力值（根据状态选择是否滤波）
    double getCurrentSensorForce() 
    {
        if (!sensor_wrench_received_) return 0.0;
        
        // 计算沿法向方向的力
        Eigen::Vector3d surface_normal = calculateSurfaceNormal();
        double normal_force = sensor_force_.dot(surface_normal);
        double raw_force = std::abs(normal_force);
        
        // 只在力控制阶段使用滤波
        if (current_state_ == ControllerState::FORCE_TUNE) 
        {
            return filterSensorForce(raw_force);
        } 
        else 
        {
            return raw_force; // 其他阶段使用原始数据
        }
    }
    
    // 重置传感器滤波器
    void resetSensorFilter() 
    {
        sensor_filter_initialized_ = false;
        filtered_sensor_force_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Sensor filter reset for force tuning");
    }
    
    // 重置力控制积分项
    void resetForceControl() 
    {
        // 重置积分相关变量

        static rclcpp::Time last_force_time = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Force control reset for new tuning phase");
    }


    
    // 计算表面法向方向
    Eigen::Vector3d calculateSurfaceNormal() 
    {
        if (sensor_wrench_received_ && sensor_force_.norm() > force_threshold_) 
        {
            // 直接使用传感器力方向作为法向
            Eigen::Vector3d normal = sensor_force_.normalized();
            
            // 可选：添加简单的滤波处理
            if (surface_normal_initialized_) 
            {
                double alpha = 0.1; // 滤波系数
                surface_normal_ = alpha * normal + (1.0 - alpha) * surface_normal_;
                surface_normal_.normalize();
            } 
            else 
            {
                surface_normal_ = normal;
                surface_normal_initialized_ = true;
            }
            
            return surface_normal_;
        }
        
        // 如果没有有效接触，返回默认法向（垂直向下）
        return Eigen::Vector3d(0, 0, 1);
    }
    
    // 计算切平面基向量
    void calculateTangentPlane(const Eigen::Vector3d& normal) 
    {
        // 世界坐标系的X和Y轴
        Eigen::Vector3d world_x(1, 0, 0);
        Eigen::Vector3d world_y(0, 1, 0);
        
        // 构建满足投影约束的切平面基向量
        // tangent_x: 在切平面内，投影到XOY平面时平行于世界X轴
        // tangent_y: 在切平面内，投影到XOY平面时平行于世界Y轴
        
        // 计算切平面内的X方向（投影到世界X轴）
        tangent_x_ = world_x - (world_x.dot(normal)) * normal;
        if (tangent_x_.norm() > 1e-6) 
        {
            tangent_x_.normalize();
        } 
        else 
        {
            // 如果法向接近世界X轴，使用世界Y轴作为切向X
            tangent_x_ = world_y - (world_y.dot(normal)) * normal;
            tangent_x_.normalize();
        }
        
        // 计算切平面内的Y方向（投影到世界Y轴）
        tangent_y_ = world_y - (world_y.dot(normal)) * normal;
        if (tangent_y_.norm() > 1e-6) 
        {
            tangent_y_.normalize();
        } 
        else 
        {
            // 如果法向接近世界Y轴，使用世界X轴作为切向Y
            tangent_y_ = world_x - (world_x.dot(normal)) * normal;
            tangent_y_.normalize();
        }
        
        // 确保tangent_x_和tangent_y_正交
        tangent_y_ = tangent_y_ - (tangent_y_.dot(tangent_x_)) * tangent_x_;
        tangent_y_.normalize();
    }
    
    // 验证投影约束
    bool validateProjectionConstraint() 
    {
        // 计算投影到XOY平面的向量
        Eigen::Vector3d projection_x = tangent_x_ - tangent_x_.dot(Eigen::Vector3d(0, 0, 1)) * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d projection_y = tangent_y_ - tangent_y_.dot(Eigen::Vector3d(0, 0, 1)) * Eigen::Vector3d(0, 0, 1);
        
        // 检查投影是否平行于对应的世界轴
        double x_alignment = std::abs(projection_x.dot(Eigen::Vector3d(1, 0, 0)));
        double y_alignment = std::abs(projection_y.dot(Eigen::Vector3d(0, 1, 0)));
        
        // 如果对齐度大于0.9，认为约束满足
        bool constraint_satisfied = (x_alignment > 0.9) && (y_alignment > 0.9);
        
        // 调试输出
        static int debug_counter = 0;
        if (debug_counter++ % 100 == 0) 
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Projection constraint: x_alignment=%.3f, y_alignment=%.3f, satisfied=%s", 
                       x_alignment, y_alignment, constraint_satisfied ? "true" : "false");
        }
        
        return constraint_satisfied;
    }
    
    // 计算切平面内的运动方向
    Eigen::Vector3d calculateTangentMotionDirection(bool is_x_motion) 
    {
        // 更新表面法向和切平面
        Eigen::Vector3d normal = calculateSurfaceNormal();
        calculateTangentPlane(normal);
        
        // 根据运动类型选择方向
        if (is_x_motion) 
        {
            return tangent_x_;
        } 
        else 
        {
            return tangent_y_;
        }
    }
    
    // 验证曲面扫描参数
    void validateSurfaceScanningParameters() 
    {
        if (sensor_wrench_received_ && sensor_force_.norm() > force_threshold_) 
        {
            Eigen::Vector3d normal = calculateSurfaceNormal();
            calculateTangentPlane(normal);
            
            // 验证投影约束
            bool constraint_satisfied = validateProjectionConstraint();
            
            // 验证切平面基向量的正交性
            double orthogonality = std::abs(tangent_x_.dot(tangent_y_));
            
            // 验证法向与切向的正交性
            double normal_tangent_x_ortho = std::abs(normal.dot(tangent_x_));
            double normal_tangent_y_ortho = std::abs(normal.dot(tangent_y_));
            bool normal_orthogonality_satisfied = (normal_tangent_x_ortho < 0.1) && (normal_tangent_y_ortho < 0.1);
            
            // 输出验证结果
            static int debug_counter = 0;
            if (debug_counter++ % 200 == 0) 
            {
                RCLCPP_INFO(this->get_logger(), 
                           "Surface scanning validation: normal=[%.3f,%.3f,%.3f], tangent_x=[%.3f,%.3f,%.3f], tangent_y=[%.3f,%.3f,%.3f]", 
                           normal.x(), normal.y(), normal.z(),
                           tangent_x_.x(), tangent_x_.y(), tangent_x_.z(),
                           tangent_y_.x(), tangent_y_.y(), tangent_y_.z());
                
                RCLCPP_INFO(this->get_logger(), 
                           "Validation results: projection_constraint=%s, orthogonality=%.3f, normal_ortho=%s", 
                           constraint_satisfied ? "OK" : "FAIL",
                           orthogonality,
                           normal_orthogonality_satisfied ? "OK" : "FAIL");
            }
        }
    }
    

    
    // 检查力是否稳定在目标范围内
    bool isForceStable() 
    {
        if (!sensor_wrench_received_) return false;
        
        double current_force = std::abs(getCurrentSensorForce());
        double force_lower = target_force_ - force_tolerance_;
        double force_upper = target_force_ + force_tolerance_;
        
        return (current_force >= force_lower && current_force <= force_upper);
    }
    
    // 计算切向运动目标位置
    Eigen::Vector3d calculateTangentialTarget() 
    {
        Eigen::Vector3d target = current_scan_position_;
        
        // 如果正在转弯，返回当前位置（避免X方向移动）
        if (scan_state_.is_turning) 
        {
            return target;
        }
        
        // 检查是否已经到达边界
        if (isAtScanBoundary()) 
        {
            // 如果到达边界，保持当前X坐标不变
            scan_state_.is_at_boundary = true;
            RCLCPP_INFO(this->get_logger(), "At boundary, keeping X position: %.3f", target.x());
            return target;
        }
        
        // 正常切向运动 - 沿世界坐标X方向运动
        if (isForwardDirection(scan_state_.current_line)) 
        {
            target.x() += tangential_step_; // 正向X方向移动
        } 
        else 
        {
            target.x() -= tangential_step_; // 反向X方向移动
        }
        
        // 检查是否超出边界
        if (target.x() > scan_params_.x_max) 
        {
            target.x() = scan_params_.x_max;
            scan_state_.is_at_boundary = true;
        } 
        else if (target.x() < scan_params_.x_min) 
        {
            target.x() = scan_params_.x_min;
            scan_state_.is_at_boundary = true;
        }
        
        // 调试输出
        static int debug_counter = 0;
        if (debug_counter++ % 50 == 0) 
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Tangential motion (world X): target_x=%.3f, step=%.3f, direction=%s", 
                       target.x(), tangential_step_,
                       isForwardDirection(scan_state_.current_line) ? "forward" : "backward");
        }
        
        return target;
    }
    
    // 计算重新接触目标位置
    Eigen::Vector3d calculateRetouchTarget() 
    {
        Eigen::Vector3d target = current_scan_position_;
        target.z() -= retouch_step_; // 向下移动
        return target;
    }
    
    void printScanningProgress() {
        RCLCPP_INFO(this->get_logger(), 
                   "Scanning progress: Line %d/%d, Direction: %s, Y=%.3f", 
                   scan_state_.current_line + 1, scan_state_.total_lines,
                   isForwardDirection(scan_state_.current_line) ? "Forward" : "Backward",
                   getCurrentY(scan_state_.current_line));
    }

    // 记录当前扫描点
    void recordScanPoint() {
        if (!ee_pose_received_) return;
        
        // 记录位置
        point_cloud_data_.positions.push_back(ee_position_);
        
        // 记录力和力矩
        if (sensor_wrench_received_) {
            point_cloud_data_.forces.push_back(sensor_force_);
            point_cloud_data_.torques.push_back(sensor_torque_);
        } else {
            point_cloud_data_.forces.push_back(Eigen::Vector3d::Zero());
            point_cloud_data_.torques.push_back(Eigen::Vector3d::Zero());
        }
        
        // 记录时间戳
        point_cloud_data_.timestamps.push_back(this->now().seconds());
        
        // 记录扫描线索引
        point_cloud_data_.line_indices.push_back(scan_state_.current_line);
        
        // 每100个点输出一次进度
        if (point_cloud_data_.positions.size() % 1000 == 0) 
        {
            RCLCPP_INFO(this->get_logger(), "Recorded %zu scan points", point_cloud_data_.positions.size());
            RCLCPP_WARN(this->get_logger(), "Position ee: [%.4f, %.4f, %.4f]", ee_position_(0), ee_position_(1), ee_position_(2));
            RCLCPP_WARN(this->get_logger(), "Position desire: [%.4f, %.4f, %.4f]", position_d_(0), position_d_(1), position_d_(2));
        }
        
    }

    // 保存点云数据到文件
    void savePointCloudData() 
    {
        if (point_cloud_data_.positions.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "No scan points to save!");
            return;
        }
        
        // 生成文件名（包含时间戳）
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        // 保存到与exp目录同级的point_cloud目录
        //std::string point_cloud_dir = "../point_cloud";
        std::string point_cloud_dir = "/home/mscrobotics2425laptop30/yxc/Project_HEUS/ros2_mujoco_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/point_cloud";
        
        ss << point_cloud_dir << "/scan_pointcloud_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        //ss << point_cloud_dir << "/scan_pointcloud.csv";
        point_cloud_data_.filename = ss.str();
        
        // 确保point_cloud目录存在
        try {
            if (!std::filesystem::exists(point_cloud_dir)) {
                std::filesystem::create_directories(point_cloud_dir);
                RCLCPP_INFO(this->get_logger(), "Created directory: %s", point_cloud_dir.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s", point_cloud_dir.c_str(), e.what());
        }
        
        // 打开文件
        std::ofstream file(point_cloud_data_.filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", point_cloud_data_.filename.c_str());
            return;
        }
        
        // 写入CSV头部
        file << "timestamp,line_index,x,y,z,force_x,force_y,force_z,torque_x,torque_y,torque_z\n";
        
        // 写入数据
        for (size_t i = 0; i < point_cloud_data_.positions.size(); ++i) {
            const auto& pos = point_cloud_data_.positions[i];
            const auto& force = point_cloud_data_.forces[i];
            const auto& torque = point_cloud_data_.torques[i];
            
            file << std::fixed << std::setprecision(6)
                 << point_cloud_data_.timestamps[i] << ","
                 << point_cloud_data_.line_indices[i] << ","
                 << pos.x() << "," << pos.y() << "," << pos.z() << ","
                 << force.x() << "," << force.y() << "," << force.z() << ","
                 << torque.x() << "," << torque.y() << "," << torque.z() << "\n";
        }
        
        file.close();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Point cloud data saved: %s (%zu points)", 
                   point_cloud_data_.filename.c_str(), 
                   point_cloud_data_.positions.size());
    }

    // 保存为PCL点云格式（可选）
    void savePCLPointCloud() {
        if (point_cloud_data_.positions.empty()) return;
        
        std::string pcl_filename = point_cloud_data_.filename;
        pcl_filename.replace(pcl_filename.find(".csv"), 4, ".pcd");
        
        // 这里可以添加PCL库的保存代码
        // 需要包含PCL头文件和链接PCL库
        RCLCPP_INFO(this->get_logger(), "PCL format not implemented yet. Use CSV format.");
    }

    // 清理点云数据
    void clearPointCloudData() {
        point_cloud_data_.positions.clear();
        point_cloud_data_.forces.clear();
        point_cloud_data_.torques.clear();
        point_cloud_data_.timestamps.clear();
        point_cloud_data_.line_indices.clear();
        point_cloud_data_.filename.clear();
        RCLCPP_INFO(this->get_logger(), "Point cloud data cleared.");
    }

    // **计算五次多项式系数的函数**
    void calculateQuinticPolynomialCoefficients(const Eigen::Vector3d& q_start, const Eigen::Vector3d& q_target, double T) 
    {
        // 假设起始和结束速度、加速度均为零
        // 对于每个维度 (x, y, z)
        for (int i = 0; i < 3; ++i) {
            double q_s = q_start(i);
            double q_t = q_target(i);
            double delta_q = q_t - q_s;

            // 五次多项式系数的闭式解 (当v_start=a_start=v_target=a_target=0)
            quintic_coeffs_(0, i) = q_s;
            quintic_coeffs_(1, i) = 0.0;
            quintic_coeffs_(2, i) = 0.0;
            quintic_coeffs_(3, i) = 10.0 * delta_q / std::pow(T, 3);
            quintic_coeffs_(4, i) = -15.0 * delta_q / std::pow(T, 4);
            quintic_coeffs_(5, i) = 6.0 * delta_q / std::pow(T, 5);
        }
        RCLCPP_INFO(this->get_logger(), "Quintic polynomial coefficients calculated.");
        total_motion_time_ = T;
    }

    //update position_d_
    void update_position_d_() 
    {

        // 计算当前在轨迹上的时间
        double current_time_in_traj = (this->now() - trajectory_start_rcl_time_).seconds();
        // 将时间限制在轨迹总时长内，确保到达终点后 position_d_ 保持在 final_target_position_
        current_time_in_traj = std::min(current_time_in_traj, total_motion_time_); 

        // 根据五次多项式更新 position_d_
        for (int i = 0; i < 3; ++i) 
        { // 针对 x, y, z 维度分别计算
            double t = current_time_in_traj;
            position_d_(i) = quintic_coeffs_(0, i) + quintic_coeffs_(1, i) * t + quintic_coeffs_(2, i) * std::pow(t, 2) +
                             quintic_coeffs_(3, i) * std::pow(t, 3) + quintic_coeffs_(4, i) * std::pow(t, 4) +
                             quintic_coeffs_(5, i) * std::pow(t, 5);
        }

        return;
    }
    
    // 使用梯形轨迹更新期望位置的函数
    void update_position_d_trapezoidal() 
    {
        if (!trapezoidal_trajectory_.is_initialized) 
        {
            return;
        }

        double current_time = (this->now() - trapezoidal_trajectory_.start_time).seconds();
        
        // 使用梯形轨迹计算期望位置
        Eigen::Vector3d desired_position = trapezoidal_trajectory_.getPositionAtTime(current_time);
        
        position_d_ = desired_position;
        /*
        // 计算表面法向方向
        Eigen::Vector3d surface_normal = calculateSurfaceNormal();
        
        // 将期望位置投影到切平面上，保持法向方向不变
        // 计算当前position_d_在法向方向上的投影
        double current_normal_projection = position_d_.dot(surface_normal);
        
        // 将梯形轨迹的期望位置投影到切平面
        double desired_normal_projection = desired_position.dot(surface_normal);
        Eigen::Vector3d desired_tangent_component = desired_position - desired_normal_projection * surface_normal;
        
        position_d_ = desired_tangent_component + current_normal_projection * surface_normal;
        */
    }

    bool initializeKDL()
    {
        //获取 robot_description 参数
        std::string robot_description_string;
        declare_parameter("robot_description", "");
        get_parameter("robot_description", robot_description_string);

        if (!kdl_parser::treeFromString(robot_description_string, kdl_tree_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF to KDL Tree.");
            return false;
        }

        // 使用 fr3_link0 作为基座，fr3_link7 作为末端（假设fr3_link7是末端执行器控制的目标帧）
        std::string base_link = "fr3_link0";
        std::string tip_link = "fr3_link7"; 
        if (!kdl_tree_.getChain(base_link, tip_link, kdl_chain_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL Chain from %s to %s", base_link.c_str(), tip_link.c_str());
            return false;
        }

        //初始化解算器
        jacobian_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
        if (!jacobian_solver_) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create Jacobian solver");
            return false;
        }

        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
        if (!fk_solver_) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create FK solver");
            return false;
        }

        return true;
    }

    void jacobianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {

        if (msg->layout.dim.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "Received Jacobian has unexpected dimensions. Expected 2 dims (rows, cols).");
            return;
        }

        size_t rows = msg->layout.dim[0].size;
        size_t cols = msg->layout.dim[1].size;


        if (rows != 6 || cols != num_joints_) { // 假设 num_joints_ 已经正确设置为7
            RCLCPP_ERROR(this->get_logger(), "Received Jacobian has incorrect size. Expected 6x%zu, got %zux%zu.", num_joints_, rows, cols);
            return;
        }

        if (msg->data.size() != rows * cols) {
            RCLCPP_ERROR(this->get_logger(), "Received Jacobian data size mismatch. Expected %zu, got %zu.", rows * cols, msg->data.size());
            return;
        }

        //RCLCPP_WARN(this->get_logger(), "======================debug point2 =========================");

        // 将扁平化的数据复制到 Eigen 矩阵
        // 注意：根据你发布的顺序，先是 jacp_ (3xN)，然后是 jacr_ (3xN)
        // Eigen 的 Map 可以直接从数组创建矩阵视图
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            jac_map(msg->data.data(), rows, cols); // 使用 RowMajor

        // 将雅可比数据赋值给成员变量
        jacobian_ = jac_map; // 或者 jacobian_ = jac_map; 如果 jacobian_ 已经是 6x7 且是 ColMajor

        jacobian_received_ = true;
    }

    // ======== 新增：末端位姿回调函数实现 ========
    void eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 提取位置信息
        ee_position_(0) = msg->pose.position.x;
        ee_position_(1) = msg->pose.position.y;
        ee_position_(2) = msg->pose.position.z;

        // 提取姿态信息
        ee_orientation_.w() = msg->pose.orientation.w;
        ee_orientation_.x() = msg->pose.orientation.x;
        ee_orientation_.y() = msg->pose.orientation.y;
        ee_orientation_.z() = msg->pose.orientation.z;

        ee_pose_received_ = true;
    }

    // ======== 新增：传感器数据回调函数实现 ========
    void sensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        // 提取力数据
        sensor_force_(0) = msg->wrench.force.x;
        sensor_force_(1) = msg->wrench.force.y;
        sensor_force_(2) = msg->wrench.force.z;

        // 提取力矩数据
        sensor_torque_(0) = msg->wrench.torque.x;
        sensor_torque_(1) = msg->wrench.torque.y;
        sensor_torque_(2) = msg->wrench.torque.z;

        sensor_wrench_received_ = true;
    }

    // ======== 新增：重力补偿回调函数实现 ========
    void gravityCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != num_joints_) 
        {
            RCLCPP_WARN(this->get_logger(), "Received gravity forces message has incorrect size (%zu).", msg->data.size());
            gravity_compensation_received_ = false; // 标记为未接收到有效数据
            return;
        }

        for (size_t i = 0; i < num_joints_; ++i) 
        {
            tau_gravity_compensation_(i) = msg->data[i] ;
        }
        gravity_compensation_received_ = true;
        
        // 调试：打印接收到的重力补偿数据
        /*
        static int gravity_debug_counter = 0;
        if (gravity_debug_counter++ % 100 == 0) {
            RCLCPP_WARN(this->get_logger(), "Received gravity compensation: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                       msg->data[4], msg->data[5], msg->data[6]);
        }
        */
    }
    

    void getCurrentPositionOrientation(Eigen::Isometry3d& transform_current
        ,const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {

        // 1. 构造当前关节状态的 KDL 类型和 Eigen 类型
        KDL::JntArray joint_positions_kdl(num_joints_);
        KDL::JntArray joint_velocities_kdl(num_joints_);
        // 直接映射到 Eigen::Matrix，方便后续计算
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(msg->position.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_current(msg->velocity.data());

        for (size_t i = 0; i < num_joints_; ++i) 
        {
            joint_positions_kdl(i) = msg->position[i];
            joint_velocities_kdl(i) = msg->velocity[i];
        }   

        // 2. 计算当前末端执行器位姿
        KDL::Frame current_pose_kdl;
        if (fk_solver_->JntToCart(joint_positions_kdl, current_pose_kdl) < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "FK failed.");
            return;
        }
        
        // **将KDL::Frame转换为Eigen::Isometry3d以获取位置和四元数姿态**
        // 手动将 KDL::Frame 转换为 Eigen::Isometry3d
        Eigen::Matrix3d rotation_matrix_current;
        for (int i = 0; i < 3; ++i) 
        {
            for (int j = 0; j < 3; ++j) 
            {
                rotation_matrix_current(i, j) = current_pose_kdl.M(i, j); // KDL::Rotation 的元素
            }
        }
        Eigen::Vector3d position_vector_current;
        position_vector_current << current_pose_kdl.p.x(), current_pose_kdl.p.y(), current_pose_kdl.p.z(); // KDL::Vector 的元素

        transform_current = Eigen::Isometry3d::Identity(); // 初始化为单位变换
        transform_current.rotate(rotation_matrix_current); // 设置旋转部分
        transform_current.pretranslate(position_vector_current); // 设置平移部分

        
    }

        void getEEPositionOrientation(Eigen::Isometry3d& transform_current)
    {

        transform_current = Eigen::Isometry3d::Identity(); // 初始化为单位变换
        transform_current.linear() = ee_orientation_.toRotationMatrix(); // 设置旋转部分（rotation）
        transform_current.translation() = ee_position_;                   // 设置平移部分（translation）

    }


    //计算力矩
    void computeAndPublishImpedanceTau(Eigen::Vector3d position_d, Eigen::Quaterniond orientation_d, 
        const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        // 直接映射到 Eigen::Matrix，方便后续计算
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(msg->position.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_current(msg->velocity.data());


        //获取当前位置
        Eigen::Isometry3d transform_current = Eigen::Isometry3d::Identity(); // 初始化为单位变换
        getEEPositionOrientation(transform_current);

        Eigen::Vector3d position_current = transform_current.translation();
        Eigen::Quaterniond orientation_current(transform_current.rotation());



        // 3. **计算6维的期望位姿误差**
        Eigen::Matrix<double, 6, 1> error;

        // 位置误差 (3维)
        error.head(3) << position_current - position_d;

        // 姿态误差 (3维 - 使用"差分"四元数并转换为旋转向量形式)
        // 确保四元数在同一个半空间，以获得最短旋转路径
        if (orientation_d.coeffs().dot(orientation_current.coeffs()) < 0.0) 
        {
            orientation_current.coeffs() << -orientation_current.coeffs();
        }
        // 计算从当前姿态到期望姿态的"差分"四元数
        Eigen::Quaterniond error_quaternion(orientation_current.inverse() * orientation_d);
        // 将四元数的虚部 (x, y, z) 作为旋转向量的近似，对于小角度误差非常有效
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // 将旋转误差从当前末端坐标系转换到基坐标系，并取负号以确保误差方向与纠正方向一致
        error.tail(3) << -transform_current.rotation() * error.tail(3); 



        // 调试输出误差
        //RCLCPP_WARN(this->get_logger(), "Current Position: [%.4f, %.4f, %.4f]", position_current(0), position_current(1), position_current(2));
       // RCLCPP_WARN(this->get_logger(), "Position desire: [%.4f, %.4f, %.4f]", position_d(0), position_d(1), position_d(2));
        //RCLCPP_WARN(this->get_logger(), "Position Error: [%.4f, %.4f, %.4f]", error(0), error(1), error(2));
        //RCLCPP_WARN(this->get_logger(), "Orientation Error: [%.4f, %.4f, %.4f]", error(3), error(4), error(5));


        // 4. 获取完整的Jacobian (6x7)
        /*
        KDL::Jacobian jacobian_kdl(num_joints_);
        if (jacobian_solver_->JntToJac(joint_positions_kdl, jacobian_kdl) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Jacobian computation failed.");
            return;
        }
        */
        
        // KDL::Jacobian 的 .data 已经是 Eigen::MatrixXd 类型
        //Eigen::MatrixXd J = jacobian_kdl.data; // 完整的 6x7 雅可比矩阵

                
        ///////////////////////////////////////////////////////////////////////////////////////
        // 检查是否已接收到雅可比
        if (!jacobian_received_) 
        {
            static int jacobian_wait_counter = 0;
            if (jacobian_wait_counter++ % 50 == 0) {
                RCLCPP_WARN(this->get_logger(), "Jacobian not yet received. Waiting... (counter: %d)", jacobian_wait_counter);
            }
            return;
        }

        // 直接使用存储的雅可比矩阵
        //Eigen::MatrixXd J = jacobian_kdl.data; // jacobian_; // 将成员变量赋值给局部变量 J，方便后续计算
        Eigen::MatrixXd J = jacobian_; 
        ///////////////////////////////////////////////////////////////////////////////////////

        
        // 奇异点检测：使用SVD计算最小奇异值，更鲁棒
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double min_singular_value = svd.singularValues().tail(1)(0);
        double max_singular_value = svd.singularValues()(0);
        double cond_number = max_singular_value / min_singular_value; // 条件数

        if (min_singular_value < 0.001) // 降低阈值，允许更接近奇异的情况
        {
            //RCLCPP_WARN(this->get_logger(), "Jacobian is near-singular (min_sv: %.4f), condition number: %.2f. Using damped least squares.", min_singular_value, cond_number);
            
            // 使用阻尼最小二乘法而不是停止
            double lambda = 0.01; // 阻尼因子
            J = J + lambda * Eigen::MatrixXd::Identity(J.rows(), J.cols());
        }
        
        


        // 5. 计算笛卡尔速度 dx_cartesian = J * dq
        Eigen::VectorXd dx_cartesian = J * dq_current; // 6维笛卡尔速度（线速度+角速度）



        // 对笛卡尔速度进行低通滤波
        Eigen::VectorXd dx_cartesian_filtered;
        if(!dx_initialized_)
        {
            dx_cartesian_filtered = dx_cartesian;
            dx_initialized_ = true;
        }
        else
        {
            dx_cartesian_filtered = 0.01* alpha * dx_cartesian + (1 - alpha) * prev_dx_cartesian_;
        }
        prev_dx_cartesian_ = dx_cartesian_filtered;



        // 6. 计算任务空间力矩 (笛卡尔阻抗控制)
        // tau_task = J^T * (-K_c * error - D_c * dx_filtered)
        Eigen::VectorXd tau_task = J.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * dx_cartesian_filtered);
        
        // 7. **核心修改点：计算零空间力矩**
        // 使用SVD计算伪逆，更鲁棒
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_for_pinv(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singular_values = svd_for_pinv.singularValues();
        
        // 设置奇异值阈值
        double threshold = 1e-6;
        for (int i = 0; i < singular_values.size(); ++i) {
            if (singular_values(i) > threshold) {
                singular_values(i) = 1.0 / singular_values(i);
            } else {
                singular_values(i) = 0.0;
            }
        }
        
        // 计算伪逆
        Eigen::MatrixXd J_pinv = svd_for_pinv.matrixV() * singular_values.asDiagonal() * svd_for_pinv.matrixU().transpose();
        
        // 零空间投影矩阵 N = I - J_pinv * J
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(num_joints_, num_joints_) - J_pinv * J;

        Eigen::VectorXd tau_nullspace = N * (nullspace_stiffness_ * (q_d_nullspace_ - q_current) -
                                             (2.0 * sqrt(nullspace_stiffness_)) * dq_current);

        // 8. 加入科里奥利力补偿 (此处为占位符，若有机器人动力学模型可提供实际值)
        // 如果你的MuJoCo仿真能够提供科里奥利力，可以在此处获取并加入。
        //Eigen::VectorXd coriolis_compensations = Eigen::VectorXd::Zero(num_joints_); 

        // 确保已接收到重力补偿力矩，否则跳过本轮控制循环
        if (!gravity_compensation_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Gravity compensation torques not yet received. Skipping control loop iteration.");
            // 你也可以选择在这里使用一个默认的零重力补偿，但这可能导致垮塌
            return;
        }



        // 9. 总的期望关节力矩 - 增加重力补偿强度
        //Eigen::VectorXd tau_d =   tau_gravity_compensation_ ;
        Eigen::VectorXd tau_d = tau_task  + tau_gravity_compensation_; // + tau_nullspace;


        // 调试：打印重力补偿力矩
        static int debug_counter = 0;
        // 调试：打印位置误差和力矩组成
        if (debug_counter % 100 == 0) {

            //RCLCPP_WARN(this->get_logger(), "Position current: [%.4f, %.4f, %.4f]", position_current(0), position_current(1), position_current(2));
            //RCLCPP_WARN(this->get_logger(), "Position ee: [%.4f, %.4f, %.4f]", ee_position_(0), ee_position_(1), ee_position_(2));
            //RCLCPP_WARN(this->get_logger(), "Position desire: [%.4f, %.4f, %.4f]", position_d(0), position_d(1), position_d(2));

            //RCLCPP_WARN(this->get_logger(), "Task torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", tau_task(0), tau_task(1), tau_task(2), tau_task(3), tau_task(4), tau_task(5), tau_task(6));
            
            /*
            RCLCPP_WARN(this->get_logger(), "Gravity compensation torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       tau_gravity_compensation_(0), tau_gravity_compensation_(1), tau_gravity_compensation_(2),
                       tau_gravity_compensation_(3), tau_gravity_compensation_(4), tau_gravity_compensation_(5),
                       tau_gravity_compensation_(6));
            */

            //RCLCPP_WARN(this->get_logger(), "Joint velocities: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", dq_current(0), dq_current(1), dq_current(2), dq_current(3), dq_current(4), dq_current(5), dq_current(6));

            // 打印传感器数据
            /*
            if (sensor_wrench_received_) 
            {
                RCLCPP_WARN(this->get_logger(), "Sensor Force: [%.3f, %.3f, %.3f] N", 
                           sensor_force_(0), sensor_force_(1), sensor_force_(2));
                RCLCPP_WARN(this->get_logger(), "Sensor Torque: [%.3f, %.3f, %.3f] Nm", 
                           sensor_torque_(0), sensor_torque_(1), sensor_torque_(2));
            }
            */

            //RCLCPP_WARN(this->get_logger(), "Current state: %s", stateToString(current_state_).c_str());

            //RCLCPP_WARN(this->get_logger(), "==================================================");
        }


        // 10. 限制力矩变化率
        // delta_tau_max 参数根据你的机器人和控制频率调整，降低以获得更平滑的控制
        Eigen::VectorXd tau_saturated = saturateTorqueRate(tau_d, tau_J_d_, 1.0); 

        // 更新上一周期发送的力矩，用于下一周期的变化率限制
        tau_J_d_ = tau_saturated;

        //tau_saturated(3) = -20;

        // 11. 发布关节力矩
        auto effort_cmd_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        effort_cmd_msg->data.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i)
        {
            effort_cmd_msg->data[i] = tau_saturated(i);
            //RCLCPP_WARN(this->get_logger(), "tau(%zu)= [%3f]", i, tau_d(i));
            //RCLCPP_WARN(this->get_logger(), "tau_saturated(%zu)= [%3f]", i, tau_saturated(i));
            //RCLCPP_WARN(this->get_logger(), "dq(%zu)= [%3f]", i, dq_current(i));
        }
        effort_command_publisher_->publish(std::move(effort_cmd_msg));




        // --- 发布当前和期望位置用于绘图 -----------------------------------------------
        auto current_pos_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        current_pos_msg->header.stamp = this->now(); // 获取当前ROS时间
        //current_pos_msg->point.x = error(0);
        //current_pos_msg->point.y = error(1);
        //current_pos_msg->point.z = error(2);

        current_pos_msg->point.x = position_current.x();
        current_pos_msg->point.y = position_current.y();
        current_pos_msg->point.z = position_current.z();

        current_position_publisher_->publish(std::move(current_pos_msg));

        auto desired_pos_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        desired_pos_msg->header.stamp = this->now();
        desired_pos_msg->point.x = position_d.x();
        desired_pos_msg->point.y = position_d.y();
        desired_pos_msg->point.z = position_d.z();

        //desired_pos_msg->point.x = dx_cartesian_filtered(0);
        //desired_pos_msg->point.y = dx_cartesian_filtered(1);
        //desired_pos_msg->point.z = dx_cartesian_filtered(2);
        desired_position_publisher_->publish(std::move(desired_pos_msg));


        // --- 发布用于绘图的力矩数据 ---
        auto published_torques_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        published_torques_msg->data.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i)
        {
            published_torques_msg->data[i] = tau_saturated(i); //-------------------
        }
        published_torques_publisher_->publish(std::move(published_torques_msg));
        // --- 发布结束 --------------------------------------------------

    
        return;
    }


    // 初始化阶段控制函数
    void stableInitialState(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr  msg)
    {

        
        // 将初始位置作为目标, 计算并发布阻抗力矩
        computeAndPublishImpedanceTau(initial_position_, initial_orientation_, msg);
     

        // 判断所有关节速度是否稳定
        bool all_below_threshold = true;
        for (size_t i = 0; i < msg->velocity.size(); ++i)
        {
            if (std::abs(msg->velocity[i]) > stable_velocity_threshold_)
            {
                all_below_threshold = false;
                break;
            }
        }
        

        // 若稳定则开始计时
        if (all_below_threshold)
        {
            if (!is_velocity_stable_)
            {
                velocity_stable_start_time_ = this->now();
                is_velocity_stable_ = true;
            }
            else
            {
                // 中文：若持续稳定超过阈值时间，进入正常控制
                if ((this->now() - velocity_stable_start_time_).seconds() > stable_hold_duration_sec_)
                {
                    current_state_ = ControllerState::APPROACH_OBJECT;
                    RCLCPP_INFO(this->get_logger(), "Stable state achieved. Switching to normal control.");
                }
            }
        }
        else
        {
            // 不稳定，重置标志
            is_velocity_stable_ = false;
        }

        
    }

    void approachObject(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
      

        // 确保接收到的关节数量与预期的相符
        if (msg->name.size() != num_joints_ ||
            msg->position.size() != num_joints_ ||
            msg->velocity.size() != num_joints_)
        {
            RCLCPP_WARN(this->get_logger(), "Received JointStateArray message has unexpected number of joints. Expected %zu, got %zu.", num_joints_, msg->name.size());
            return;
        }


        // --- 轨迹规划准备 ---
        if (!trajectory_planning_started_) 
        {
            
            //获取当前位置
            Eigen::Isometry3d transform_current = Eigen::Isometry3d::Identity(); // 初始化为单位变换
            getEEPositionOrientation(transform_current);

            Eigen::Vector3d position_current = transform_current.translation();
            Eigen::Quaterniond orientation_current(transform_current.rotation());

        
            // ************************************************************
            trajectory_start_position_ = position_current; // 记录轨迹规划开始时机器人的当前位置

            // *** 将期望姿态设置为当前姿态 ***
            position_d_ = position_current;
            orientation_d_ = orientation_current; // 这会确保姿态保持不变

            // ************************************************************

            trajectory_start_rcl_time_ = this->now(); // 记录轨迹开始的时间
            calculateQuinticPolynomialCoefficients(trajectory_start_position_, final_target_position_, total_motion_time_);

            trajectory_planning_started_ = true;

        }
        // --- 轨迹规划准备结束 ---

        //position_d_ =  Eigen::Vector3d(0.3, 0.0, 0.5);
        // 平滑更新期望位置，目标是 final_target_position_
        //position_d_ = filter_params_ * final_target_position_ + (1.0 - filter_params_) * position_d_;
        update_position_d_();  


        //*******************************compute and publish tau ****************************************** */
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);


        //*************************************** 判断下一阶段 ********************************************************** */
        // 检测是否接触到物体
        if (sensor_wrench_received_) 
        {
            double force_magnitude = std::abs(sensor_force_.z());
            
            if (force_magnitude > contact_force_threshold_) 
            {
                RCLCPP_INFO(this->get_logger(), "Contact detected! Force magnitude: %.3f N. Transitioning to contact confirmation.", force_magnitude);
                 
                // 清理之前的点云数据，开始新的扫描记录
                clearPointCloudData();
                 
                current_state_ = ControllerState::CONTACT_CONFIRM;
                force_stable_start_time_ = this->now();
                is_force_stable_ = false;
                
                // 启动扫描计时器（如果还没启动）
                if (!scanning_timer_started_) 
                {
                    scanning_start_time_ = this->now();
                    scanning_timer_started_ = true;
                }
                
                // 重置轨迹规划标志，为扫描阶段准备
                trajectory_planning_started_ = false;
            }
        }
        

    }

    
    void contactConfirmControl(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        // 保持当前位置,先不更新p_d，等待接触确认
        //if (ee_pose_received_) 
        {
        //    position_d_ = ee_position_;
        }
        
        // 执行阻抗控制
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);
        
        // 检查接触力
        if (sensor_wrench_received_) 
        {
            // 计算沿法向方向的力
            Eigen::Vector3d surface_normal = calculateSurfaceNormal();
            double contact_force = std::abs(sensor_force_.dot(surface_normal));
            
            if (contact_force > contact_force_threshold_) 
            {
                // 检测到接触
                if (!is_force_stable_) 
                {
                    force_stable_start_time_ = this->now();
                    is_force_stable_ = true;
                    contact_loss_count_ = 0; // 重置接触丢失计数器
                    RCLCPP_INFO(this->get_logger(), "Contact detected! Force: %.3f N. Starting contact confirmation...", contact_force);
                } 
                else 
                {
                    // 检查接触是否稳定足够长时间
                    double contact_time = (this->now() - force_stable_start_time_).seconds();
                    if (contact_time >= 0.5 ) //force_stable_time_) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Contact confirmed! Transitioning to force tuning...");
                        current_state_ = ControllerState::FORCE_TUNE;
                        current_scan_position_ = ee_position_;
                        scan_start_position_ = ee_position_;
                        is_force_stable_ = false;
                        contact_loss_count_ = 0; // 重置接触丢失计数器
                        resetSensorFilter(); // 重置传感器滤波器
                        resetForceControl(); // 重置力控制积分项
                    }
                }
            } 
            else 
            {
                // 接触丢失，增加计数器
                contact_loss_count_++;
                is_force_stable_ = false;

                RCLCPP_INFO(this->get_logger(), "Contact confirmed failed! Contact loss count: %d/%d", 
                           contact_loss_count_, max_contact_loss_count_);
                
                // 如果接触丢失次数达到阈值，转换到retouch阶段
                if (contact_loss_count_ >= max_contact_loss_count_) 
                {
                    RCLCPP_WARN(this->get_logger(), "Contact lost %d times! Transitioning to retouch...", contact_loss_count_);
                    current_state_ = ControllerState::RETOUCH;
                    contact_loss_count_ = 0; // 重置计数器
                    trajectory_planning_started_ = false;
                    return;
                }
            }
        }


    }
    
    void forceTuneControl(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {

        /*
        
        // 力调整逻辑
        if (sensor_wrench_received_) 
        {
            // 获取当前传感器力值（在力控制阶段自动使用滤波）
            double current_force = getCurrentSensorForce();
            double force_error = current_force - target_force_;
            
            // 死区控制：如果力误差在死区内，不进行调整
            if (std::abs(force_error) <= force_deadzone_) 
            {
                force_error = 0.0; // 将死区内的误差置零
            }
            
            // 计算表面法向方向
            Eigen::Vector3d surface_normal = calculateSurfaceNormal();
            
            // 基于阻抗控制器刚度计算导纳增益
            double z_stiffness = cartesian_stiffness_(2, 2);  // Z方向刚度 (N/m)
            double damping_ratio = 0.7;  // 阻尼比
            double admittance_gain = 1.0 / (z_stiffness * damping_ratio);
            
            // 力误差积分（带抗积分饱和）
            static double force_integral = 0.0;
            static rclcpp::Time last_force_time = this->now();
            static bool integral_initialized = false;
            
            double dt = (this->now() - last_force_time).seconds();
            if (dt > 0.0) 
            {
                if (!integral_initialized) 
                {
                    force_integral = 0.0;
                    integral_initialized = true;
                }
                
                // 积分项（带抗积分饱和）- 只在死区外积分
                if (std::abs(force_error) > force_deadzone_) 
                {
                    force_integral += force_error * dt;
                    force_integral = std::clamp(force_integral, -50.0, 50.0); // 限制积分范围
                }
                
                // 计算沿法向方向的调整量（比例 + 积分）
                double integral_gain = 0.1; // 积分增益
                double adjustment_magnitude = admittance_gain * (force_error + integral_gain * force_integral);
                
                // 限制调整幅度
                adjustment_magnitude = std::clamp(adjustment_magnitude, -0.001, 0.001);
                
                // 沿法向方向调整位置
                Eigen::Vector3d position_adjustment = surface_normal * adjustment_magnitude;
                
                // 渐进式调整（低通滤波）
                static Eigen::Vector3d filtered_position_target = position_d_;
                double alpha = 0.05; // 滤波系数
                filtered_position_target = alpha * (position_d_ + position_adjustment) + (1.0 - alpha) * filtered_position_target;
                position_d_ = filtered_position_target;
                
                last_force_time = this->now();
                
                // 调试输出（可选）
                static int debug_counter = 0;
                if (debug_counter++ % 100 == 0) {
                    // 计算沿法向方向的原始力
                    double raw_force = std::abs(sensor_force_.dot(surface_normal));
                    RCLCPP_INFO(this->get_logger(), 
                               "Force tuning: raw_force=%.2f N, filtered_force=%.2f N, error=%.2f N, integral=%.2f, adj_mag=%.6f m, normal=[%.3f,%.3f,%.3f], pos=[%.6f,%.6f,%.6f]", 
                               raw_force, current_force, force_error, force_integral, adjustment_magnitude, 
                               surface_normal.x(), surface_normal.y(), surface_normal.z(),
                               position_d_.x(), position_d_.y(), position_d_.z());
                }
            }
            
            // 检查力是否稳定在目标范围内
            if (isForceStable()) 
            {
                if (!is_force_stable_) 
                {
                    force_stable_start_time_ = this->now();
                    is_force_stable_ = true;
                    RCLCPP_INFO(this->get_logger(), "Force tuning: Force stabilized at %.2f N", current_force);
                } 
                else 
                {
                    // 检查力是否稳定足够长时间
                    double force_time = (this->now() - force_stable_start_time_).seconds();
                    if (force_time >= 0.01) //force_stable_time_) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Force tuning completed! Transitioning to tangential scan...");
                        current_state_ = ControllerState::TANGENTIAL_SCAN;
                        current_scan_position_ = ee_position_;
                        is_force_stable_ = false;
                        trajectory_planning_started_ = false;
                        
                        // 重置积分器
                        force_integral = 0.0;
                        integral_initialized = false;
                    }
                }

            } 
            else 
            {
                is_force_stable_ = false;
                //RCLCPP_INFO(this->get_logger(), "Force not stable: %.2f N", current_force);
            }
        }

        */
        
        /*
        RCLCPP_INFO(this->get_logger(), "Force tuning completed! Transitioning to tangential scan...");
        current_state_ = ControllerState::TANGENTIAL_SCAN;
        current_scan_position_ = ee_position_;
        is_force_stable_ = false;
        trajectory_planning_started_ = false;
        */       

        // 力调整逻辑
        if (sensor_wrench_received_) 
        {
            // 获取当前传感器力值（在力控制阶段自动使用滤波）
            double current_force = getCurrentSensorForce();
            double force_error = current_force - target_force_;
            
            // 死区控制：如果力误差在死区内，不进行调整
            if (std::abs(force_error) <= force_deadzone_) 
            {
                force_error = 0.0; // 将死区内的误差置零
            }
            
            // 基于阻抗控制器刚度计算导纳增益
            double z_stiffness = cartesian_stiffness_(2, 2);  // Z方向刚度 (N/m)
            double damping_ratio = 0.7;  // 阻尼比
            double admittance_gain = 1.0 / (z_stiffness * damping_ratio);
            
            // 力误差积分（带抗积分饱和）
            static double force_integral = 0.0;
            static rclcpp::Time last_force_time = this->now();
            static bool integral_initialized = false;
            
            double dt = (this->now() - last_force_time).seconds();
            if (dt > 0.0) 
            {
                if (!integral_initialized) 
                {
                    force_integral = 0.0;
                    integral_initialized = true;
                }
                
                // 积分项（带抗积分饱和）- 只在死区外积分
                if (std::abs(force_error) > force_deadzone_) 
                {
                    force_integral += force_error * dt;
                    force_integral = std::clamp(force_integral, -50.0, 50.0); // 限制积分范围
                }
                
                // 计算调整量（比例 + 积分）
                double integral_gain = 0.1; // 积分增益
                double normal_adjustment = admittance_gain * (force_error + integral_gain * force_integral);
                
                // 限制调整幅度
                normal_adjustment = std::clamp(normal_adjustment, -0.001, 0.001);
                
                // 渐进式调整（低通滤波）
                static double filtered_normal_target = position_d_.z();
                double alpha = 0.05; 
                filtered_normal_target = alpha * (position_d_.z() + normal_adjustment) + (1.0 - alpha) * filtered_normal_target;
                position_d_.z() = filtered_normal_target;
                
                last_force_time = this->now();
                
                // 调试输出（可选）
                static int debug_counter = 0;
                if (debug_counter++ % 100 == 0) 
                {
                    RCLCPP_INFO(this->get_logger(), 
                               "Force tuning: filtered_force=%.2f N, error=%.2f N, integral=%.2f, normal_adj=%.6f m", 
                               current_force, force_error, force_integral, normal_adjustment);
                }
            }
            
            // 检查力是否稳定在目标范围内
            if (isForceStable()) 
            {
                if (!is_force_stable_) 
                {
                    force_stable_start_time_ = this->now();
                    is_force_stable_ = true;
                    RCLCPP_INFO(this->get_logger(), "Force tuning: Force stabilized at %.2f N", current_force);
                } 
                else 
                {
                    // 检查力是否稳定足够长时间
                    double force_time = (this->now() - force_stable_start_time_).seconds();
                    if (force_time >= 0.01) //force_stable_time_) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Force tuning completed! Transitioning to tangential scan...");
                        current_state_ = ControllerState::TANGENTIAL_SCAN;
                        current_scan_position_ = ee_position_;
                        is_force_stable_ = false;
                        trajectory_planning_started_ = false;
                        
                        // 重置积分器
                        force_integral = 0.0;
                        integral_initialized = false;
                    }
                }

            } 
            else 
            {
                is_force_stable_ = false;
                //RCLCPP_INFO(this->get_logger(), "Force not stable: %.2f N", current_force);
            }
        }

        // 执行阻抗控制
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);

    }
    
    void tangentialScanControl(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        // 轨迹规划准备
        if (!trajectory_planning_started_) 
        {
            // 检查是否到达边界
            if (isAtScanBoundary()) 
            {
                scan_state_.is_at_boundary = true;
                RCLCPP_INFO(this->get_logger(), "Reached scan boundary. Checking if scanning is complete...");
                
                // 检查是否完成所有扫描线
                if (scan_state_.current_line >= scan_state_.total_lines - 1) 
                {
                    // 停止扫描计时器
                    if (scanning_timer_started_) 
                    {
                        scanning_end_time_ = this->now(); // 记录结束时间
                        scanning_timer_started_ = false; // 停止计时器
                        scanning_timer_stopped_ = true; // 标记计时器已停止
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "All scanning lines completed!");
                    current_state_ = ControllerState::SCANNING_COMPLETED;
                    savePointCloudData();
                    return;
                } 
                else 
                {
                    // 进入转弯状态：先保持X不变，移动Y到下一条扫描线
                    scan_state_.is_turning = true;
                    scan_state_.x_position_locked = true;
                    scan_state_.locked_x_position = ee_position_.x(); // 锁定当前X位置
                    scan_state_.target_y_position = getCurrentY(scan_state_.current_line + 1); // 下一条扫描线的Y位置
                    
                    RCLCPP_INFO(this->get_logger(), "Starting turn: keeping X=%.3f, moving Y to %.3f", 
                               scan_state_.locked_x_position, scan_state_.target_y_position);
                }
            }

            
            // 根据是否在转弯状态选择目标位置
            if (scan_state_.is_turning) 
            {
                // 转弯状态：在切平面内沿Y方向运动
                Eigen::Vector3d surface_normal = calculateSurfaceNormal();
                calculateTangentPlane(surface_normal);
                
                // 计算在切平面内沿Y方向的目标位置
                double y_step = scan_state_.target_y_position - current_scan_position_.y();
                Eigen::Vector3d y_motion_direction = tangent_y_;
                if (y_step < 0) 
                {
                    y_motion_direction = -tangent_y_; // 反向运动
                }
                
                // 在切平面内计算目标位置
                final_target_position_ = current_scan_position_ + y_motion_direction * std::abs(y_step);
                
                RCLCPP_INFO(this->get_logger(), "Turn mode (tangent plane): target = [%.3f, %.3f, %.3f], normal=[%.3f,%.3f,%.3f], tangent_y=[%.3f,%.3f,%.3f]", 
                           final_target_position_.x(), final_target_position_.y(), final_target_position_.z(),
                           surface_normal.x(), surface_normal.y(), surface_normal.z(),
                           tangent_y_.x(), tangent_y_.y(), tangent_y_.z());
            } 
            else 
            {
                // 正常扫描状态：计算切向运动目标位置
                final_target_position_ = calculateTangentialTarget();
            }

            
            // 使用梯形轨迹规划（替换原有的五次多项式轨迹）
            trapezoidal_trajectory_.initialize(ee_position_, final_target_position_);
            trapezoidal_trajectory_.start_time = this->now();
            trajectory_planning_started_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Starting tangential scan (trapezoidal): [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                       ee_position_.x(), ee_position_.y(), ee_position_.z(),
                       final_target_position_.x(), final_target_position_.y(), final_target_position_.z());
            
            // 注释掉原有的五次多项式轨迹规划
            /*
            // 规划轨迹
            trajectory_start_position_ = ee_position_;
            trajectory_start_rcl_time_ = this->now();
            calculateQuinticPolynomialCoefficients(trajectory_start_position_, final_target_position_, tangential_scan_time_);
            

            trajectory_planning_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting tangential scan: [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                       ee_position_.x(), ee_position_.y(), ee_position_.z(),
                       final_target_position_.x(), final_target_position_.y(), final_target_position_.z());
            */
        }
        
        // 更新期望位置
        // 使用梯形轨迹更新期望位置
        update_position_d_trapezoidal();
        
        // 五次多项式位置更新
        // update_position_d_();
        
        // 执行阻抗控制
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);
        
        // 记录扫描点
        recordScanPoint();
        
        // 检查轨迹是否完成
        double x_error = final_target_position_.x() - ee_position_.x();
        double y_error = final_target_position_.y() - ee_position_.y();
        double position_error = std::sqrt(x_error * x_error + y_error * y_error);
        
        if (position_error <= scan_params_.position_tolerance) 
        {
            // 更新当前扫描位置
            current_scan_position_ = position_d_;

            // 检查是否在转弯状态
            if (scan_state_.is_turning) 
            {
                    // 转弯完成，切换到下一条扫描线
                scan_state_.current_line++;
                scan_state_.is_turning = false;
                scan_state_.x_position_locked = false;
                    
                // 更新当前扫描位置到新扫描线的起始位置
                double next_y = getCurrentY(scan_state_.current_line);
                double next_x = isForwardDirection(scan_state_.current_line) ? 
                                   scan_params_.x_min : scan_params_.x_max;
                current_scan_position_ << next_x, next_y, current_scan_position_.z();
                    
                RCLCPP_INFO(this->get_logger(), "Turn completed! Moving to next scanning line: %d/%d at [%.3f, %.3f, %.3f]", 
                               scan_state_.current_line + 1, scan_state_.total_lines,
                               next_x, next_y, current_scan_position_.z());
            }
                
            
            // 检查是否离开表面
            if (isOffSurface()) 
            {
                RCLCPP_WARN(this->get_logger(), "Lost contact during tangential scan! Transitioning to retouch...");
                current_state_ = ControllerState::RETOUCH;
                trajectory_planning_started_ = false;
                trapezoidal_trajectory_.reset(); // 重置梯形轨迹
            } 
            else 
            {
                
                // 继续力调整
                RCLCPP_INFO(this->get_logger(), "Tangential scan completed (pos_error=%.4fm). Transitioning to force tuning...",  position_error);
                current_state_ = ControllerState::FORCE_TUNE;
                trajectory_planning_started_ = false;
                trapezoidal_trajectory_.reset(); // 重置梯形轨迹
                resetSensorFilter(); // 重置传感器滤波器
                resetForceControl(); // 重置力控制积分项
            }
        }

    }
    

    void retouchControl(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        // 轨迹规划准备
        if (!trajectory_planning_started_) 
        {
            // 计算重新接触目标位置
            Eigen::Vector3d retouch_target = calculateRetouchTarget();
            
            // 规划轨迹
            trajectory_start_position_ = ee_position_;
            trajectory_start_rcl_time_ = this->now();
            calculateQuinticPolynomialCoefficients(trajectory_start_position_, retouch_target, retouch_time_);
            
            trajectory_planning_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting retouch: [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                       current_scan_position_.x(), current_scan_position_.y(), current_scan_position_.z(),
                       retouch_target.x(), retouch_target.y(), retouch_target.z());
        }
        
        // 更新期望位置
        update_position_d_();
        
        // 执行阻抗控制
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);
        
        // 在每次循环中检测接触（避免过度下压）
        if (sensor_wrench_received_) 
        {
            // 计算沿法向方向的力
            Eigen::Vector3d surface_normal = calculateSurfaceNormal();
            double contact_force = std::abs(sensor_force_.dot(surface_normal));
            
            if (contact_force > contact_force_threshold_) 
            {
                RCLCPP_INFO(this->get_logger(), "Retouch successful! Transitioning to contact confirmation...");
                current_state_ = ControllerState::CONTACT_CONFIRM;
                force_stable_start_time_ = this->now();
                is_force_stable_ = false;
                contact_loss_count_ = 0; // 重置接触丢失计数器
                trajectory_planning_started_ = false;
                return;
            }
        }
        
        // 检查轨迹是否完成
        double current_time = (this->now() - trajectory_start_rcl_time_).seconds();
        if (current_time >= retouch_time_) 
        {
            // 更新当前扫描位置
            current_scan_position_ = position_d_;
            
            // 如果时间到了还没接触，继续重新接触
            RCLCPP_WARN(this->get_logger(), "Retouch time exceeded, continuing retouch...");
            trajectory_planning_started_ = false;
        }

    }
    



    // 新增离散扫描控制函数
    void scanningCompletedControl(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        // 保持当前位置
        if (ee_pose_received_) 
        {
            position_d_ = ee_position_;
        }
        
        // 执行阻抗控制以保持位置
        computeAndPublishImpedanceTau(position_d_, orientation_d_, msg);
        
        // 计算并显示扫描总时间
        if (scanning_timer_stopped_) 
        {
            double total_scanning_time = (scanning_end_time_ - scanning_start_time_).seconds();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Discrete scanning completed! Total %d lines scanned in %.2f seconds. Holding position at [%.3f, %.3f, %.3f]", 
                                scan_state_.total_lines, total_scanning_time, position_d_.x(), position_d_.y(), position_d_.z());
        } 
        else 
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Discrete scanning completed! Total %d lines scanned. Holding position at [%.3f, %.3f, %.3f]", 
                                scan_state_.total_lines, position_d_.x(), position_d_.y(), position_d_.z());
        }
    }

    // 梯形轨迹规划器结构体
    struct TrapezoidalTrajectory {
        Eigen::Vector3d start_position;    // 起始位置
        Eigen::Vector3d end_position;      // 结束位置
        double max_velocity = 0.02;        // 最大速度 (m/s)
        double max_acceleration = 0.05;    // 最大加速度 (m/s²)
        
        double total_distance = 0.0;       // 总距离
        double acceleration_time = 0.0;    // 加速时间
        double constant_velocity_time = 0.0; // 匀速时间
        double deceleration_time = 0.0;    // 减速时间
        double total_time = 0.0;           // 总时间
        
        bool is_initialized = false;       // 是否已初始化
        rclcpp::Time start_time;           // 轨迹开始时间
        
        // 初始化梯形轨迹
        void initialize(const Eigen::Vector3d& start, const Eigen::Vector3d& end) 
        {
            start_position = start;
            end_position = end;
            total_distance = (end_position - start_position).norm();
            
            if (total_distance < 1e-6) 
            {
                // 距离太小，直接设置为完成
                total_time = 0.0;
                acceleration_time = 0.0;
                constant_velocity_time = 0.0;
                deceleration_time = 0.0;
            } 
            else 
            {
                // 计算梯形轨迹参数
                // 加速段：v = a*t, s = 0.5*a*t²
                // 减速段：v = a*t, s = 0.5*a*t²
                // 匀速段：s = v*t
                
                // 假设加速和减速时间相等
                acceleration_time = max_velocity / max_acceleration;
                double acceleration_distance = 0.5 * max_acceleration * acceleration_time * acceleration_time;
                
                if (2 * acceleration_distance >= total_distance) 
                {
                    // 三角形轨迹：没有匀速段
                    acceleration_time = std::sqrt(total_distance / max_acceleration);
                    constant_velocity_time = 0.0;
                    deceleration_time = acceleration_time;
                } 
                else 
                {
                    // 梯形轨迹：有匀速段
                    deceleration_time = acceleration_time;
                    double constant_velocity_distance = total_distance - 2 * acceleration_distance;
                    constant_velocity_time = constant_velocity_distance / max_velocity;
                }
                
                total_time = acceleration_time + constant_velocity_time + deceleration_time;
            }
            
            is_initialized = true;
        }
        
        // 获取指定时间的位置
        Eigen::Vector3d getPositionAtTime(double current_time) 
        {
            if (!is_initialized || total_distance < 1e-6) 
            {
                return end_position;
            }
            
            // 限制时间范围
            current_time = std::clamp(current_time, 0.0, total_time);
            
            double distance_ratio = 0.0;
            
            if (current_time <= acceleration_time) 
            {
                // 加速段：s = 0.5*a*t²
                distance_ratio = 0.5 * max_acceleration * current_time * current_time / total_distance;
            } 
            else if (current_time <= acceleration_time + constant_velocity_time) 
            {
                // 匀速段
                double constant_time = current_time - acceleration_time;
                double acceleration_distance = 0.5 * max_acceleration * acceleration_time * acceleration_time;
                double constant_distance = max_velocity * constant_time;
                distance_ratio = (acceleration_distance + constant_distance) / total_distance;
            } 
            else 
            {
                // 减速段
                double decel_time = current_time - acceleration_time - constant_velocity_time;
                double acceleration_distance = 0.5 * max_acceleration * acceleration_time * acceleration_time;
                double constant_distance = max_velocity * constant_velocity_time;
                double decel_distance = max_velocity * decel_time - 0.5 * max_acceleration * decel_time * decel_time;
                distance_ratio = (acceleration_distance + constant_distance + decel_distance) / total_distance;
            }
            
            // 线性插值计算位置
            return start_position + distance_ratio * (end_position - start_position);
        }
        
        // 检查轨迹是否完成
        bool isCompleted(double current_time) 
        {
            return current_time >= total_time;
        }
        
        // 重置轨迹
        void reset() 
        {
            is_initialized = false;
            total_distance = 0.0;
            acceleration_time = 0.0;
            constant_velocity_time = 0.0;
            deceleration_time = 0.0;
            total_time = 0.0;
        }
    };
    
    // 梯形轨迹规划器实例
    TrapezoidalTrajectory trapezoidal_trajectory_;
    
    // 力矩变化率饱和函数 (保持不变)
    Eigen::VectorXd saturateTorqueRate(const Eigen::VectorXd& tau_d_calculated, const Eigen::VectorXd& tau_J_d, double delta_tau_max) 
    {
        if (tau_d_calculated.size() != tau_J_d.size()) {
            RCLCPP_WARN(this->get_logger(), "tau_d_calculated and tau_J_d must have the same dimension! [%zu,%zu]", tau_d_calculated.size(),tau_J_d.size());
            return tau_d_calculated; // 维度不匹配时，直接返回计算值，不进行饱和
        }

        Eigen::VectorXd tau_d_saturated(tau_J_d.size());

        for (int i = 0; i < tau_J_d.size(); ++i) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::clamp(difference, -delta_tau_max, delta_tau_max);
        }
        return tau_d_saturated;
    }

    // 主回调函数，调用控制逻辑
    void jointStateCallback(const mujoco_ros2_control_demos::msg::JointStateArray::SharedPtr msg)
    {
        if(stop) // 如果有停止标志（如奇异点），则退出
        {
            return;
        }

        //KDL运动学初始化
        if(! is_KDL_prepared_ )
        {
            if (!initializeKDL()) 
            {
                RCLCPP_ERROR(this->get_logger(), "KDL initialization failed");
                return;
            }
            else
            {
                is_KDL_prepared_ = true;
                RCLCPP_WARN(this->get_logger(), "KDL initialization succeed ******************");

                //设置初始位置
                Eigen::Isometry3d transform_current = Eigen::Isometry3d::Identity(); // 初始化为单位变换
                getCurrentPositionOrientation(transform_current,msg); //初始可能还未受到EE位置，先用KDL计算

                initial_position_ = transform_current.translation();
                initial_orientation_ = Eigen::Quaterniond(transform_current.rotation());

                RCLCPP_WARN(this->get_logger(), "initial position : [%.4f, %.4f, %.4f]", initial_position_(0), initial_position_(1), initial_position_(2));

                current_state_ = ControllerState::STABLE_INIT;
            }
        }

        
        switch (current_state_)
        {
            case ControllerState::KDL_INIT:
                break;
            case ControllerState::STABLE_INIT:
                stableInitialState(msg);
                break;
            case ControllerState::APPROACH_OBJECT:
                approachObject(msg);
                break;
            case ControllerState::CONTACT_CONFIRM:
                contactConfirmControl(msg);
                break;
            case ControllerState::FORCE_TUNE:
                forceTuneControl(msg);
                break;
            case ControllerState::TANGENTIAL_SCAN:
                tangentialScanControl(msg);
                break;
            case ControllerState::RETOUCH:
                retouchControl(msg);
                break;
            case ControllerState::SCANNING_COMPLETED:
                scanningCompletedControl(msg);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown state: %d", static_cast<int>(current_state_));
                break;
        }

        return;
    }




    rclcpp::Subscription<mujoco_ros2_control_demos::msg::JointStateArray>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_command_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        
    // 新增：雅可比矩阵订阅器
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_subscription_;
    Eigen::MatrixXd jacobian_; // 用于存储接收到的雅可比矩阵
    bool jacobian_received_ = false; // 标志位，表示是否已接收到雅可比

    
    std::vector<std::string> joint_names_;
    size_t num_joints_;

    // **新增成员变量用于期望位姿和零空间构型**
    Eigen::Vector3d position_d_; // 轨迹规划器当前提供的期望位置 (会被更新)
    Eigen::Vector3d final_target_position_; // 轨迹规划的最终目标位置
    Eigen::Quaterniond orientation_d_; // 期望姿态 (不变，直接作为最终目标)
    Eigen::Matrix<double, 7, 1> q_d_nullspace_; // 期望零空间关节构型

    // --- 轨迹规划相关成员变量 ---
    Eigen::Matrix<double, 6, 3> quintic_coeffs_; // 存储 x, y, z 各自的 6 个五次多项式系数
    rclcpp::Time trajectory_start_rcl_time_; // 轨迹开始的 ROS 时间
    double total_motion_time_; // 轨迹的总时长
    Eigen::Vector3d trajectory_start_position_; // 轨迹开始时机器人的实际位置
    bool trajectory_planning_started_ = false; // 轨迹规划是否已启动的标志

    double filter_params_  = 0.0001;
    // ----------------------------


    // **修改阻抗参数类型为6x6矩阵**
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_; // 笛卡尔刚度矩阵
    Eigen::Matrix<double, 6, 6> cartesian_damping_;   // 笛卡尔阻尼矩阵
    double nullspace_stiffness_; // 零空间刚度

    /*****************************to be delete********************************** */
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    /************************************************************************** */
    

    // 修改缓存变量为6维
    Eigen::VectorXd prev_dx_cartesian_ = Eigen::VectorXd::Zero(6);  // 缓存前一时刻的6维笛卡尔速度
    double alpha = 0.1;  // 滤波系数 0.0~1.0，越小越平滑
    bool dx_initialized_ = false; // 滤波初始化标志

    Eigen::VectorXd tau_J_d_; // 上一控制周期发送给电机的期望力矩 (已在构造函数中初始化)

    // 力矩限制常量 (保持不变，但需注意在控制循环中是否真正用于饱和)
    const std::vector<double> TORQUE_LIMITS = 
    {
        87.0,  // fr3_joint1
        87.0,  // fr3_joint2
        87.0,  // fr3_joint3
        87.0,  // fr3_joint4
        12.0,  // fr3_joint5
        12.0,  // fr3_joint6
        12.0   // fr3_joint7
    };

    bool stop =false; // 停止标志


    // ======== 新增：重力补偿相关的成员变量和订阅器 ========
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_forces_subscription_;
    Eigen::VectorXd tau_gravity_compensation_; // 存储接收到的重力补偿力矩
    bool gravity_compensation_received_ = false; // 标志位，确保在计算前接收到数据

    // ======== 新增：末端位姿相关的成员变量和订阅器 ========
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_subscription_;
    Eigen::Vector3d ee_position_; // 存储接收到的末端位置
    Eigen::Quaterniond ee_orientation_; // 存储接收到的末端姿态
    bool ee_pose_received_ = false; // 标志位，确保在计算前接收到数据

    // ======== 新增：传感器数据相关的成员变量和订阅器 ========
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_wrench_subscription_;
    Eigen::Vector3d sensor_force_; // 存储接收到的力数据
    Eigen::Vector3d sensor_torque_; // 存储接收到的力矩数据
    bool sensor_wrench_received_ = false; // 标志位，确保在计算前接收到数据

    // ======== 新增：曲面扫描相关的成员变量 ========
    Eigen::Vector3d surface_normal_; // 表面法向方向
    Eigen::Vector3d tangent_x_; // 切平面内X方向（投影到世界X轴）
    Eigen::Vector3d tangent_y_; // 切平面内Y方向（投影到世界Y轴）
    bool surface_normal_initialized_ = false; // 表面法向是否已初始化
    const double force_threshold_ = 1.0; // 力阈值，用于判断是否有效接触
    
    // ======== 新增：接触确认相关的成员变量 ========
    int contact_loss_count_ = 0; // 接触丢失计数器
    const int max_contact_loss_count_ = 10; // 最大接触丢失次数

    
    // 状态机相关变量（替代了is_InitialState_）
    bool is_velocity_stable_ = false;
    rclcpp::Time velocity_stable_start_time_;
    const double stable_velocity_threshold_ = 0.05; // 稳定速度阈值 rad/s - 降低以获得更稳定的判断
    const double stable_hold_duration_sec_ = 1.0;   // 持续时间 - 增加以确保真正稳定
            
    Eigen::Vector3d initial_position_ = Eigen::Vector3d::Zero(); 
    Eigen::Quaterniond initial_orientation_ = Eigen::Quaterniond::Identity(); 
    //Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial_;


    bool is_KDL_prepared_ = false; //KDL准备完成标志

    //发布位置,力矩 用于绘图
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr current_position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr published_torques_publisher_; 


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FR3ImpedanceController>());
    rclcpp::shutdown();
    return 0;
}