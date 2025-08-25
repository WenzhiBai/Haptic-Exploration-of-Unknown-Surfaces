// contact_point_recorder.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <vector>
#include <chrono>
#include <filesystem>

struct ContactPoint {
    double x, y, z;           // 位置
    double nx, ny, nz;        // 法向量
    double fx, fy, fz;        // 力向量
    double timestamp;          // 时间戳
    std::string frame_id;     // 坐标系
    std::string state_name;   // 状态名称
};

class ContactPointRecorder : public rclcpp::Node
{
public:
    ContactPointRecorder()
    : Node("contact_point_recorder")
    {
        // 声明参数
        this->declare_parameter<std::string>("ee_pose_topic", "/ee_pose");
        this->declare_parameter<std::string>("wrench_topic", "/touch_tip/wrench");
        this->declare_parameter<std::string>("contact_points_topic", "/contact_points");
        this->declare_parameter<std::string>("output_directory", "./contact_data");
        this->declare_parameter<double>("force_threshold", 5.0);  // 接触力阈值
        this->declare_parameter<double>("min_contact_duration", 0.1);  // 最小接触持续时间
        this->declare_parameter<bool>("enable_visualization", true);  // 是否启用可视化
        
        // 获取参数
        this->get_parameter("ee_pose_topic", ee_pose_topic_);
        this->get_parameter("wrench_topic", wrench_topic_);
        this->get_parameter("contact_points_topic", contact_points_topic_);
        this->get_parameter("output_directory", output_directory_);
        this->get_parameter("force_threshold", force_threshold_);
        this->get_parameter("min_contact_duration", min_contact_duration_);
        this->get_parameter("enable_visualization", enable_visualization_);
        
        // 创建输出目录
        createOutputDirectory();
        
        // 订阅话题
        sub_ee_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            ee_pose_topic_, 10, std::bind(&ContactPointRecorder::onEePose, this, std::placeholders::_1));
        
        sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_topic_, 10, std::bind(&ContactPointRecorder::onWrench, this, std::placeholders::_1));
        
        sub_contact_points_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            contact_points_topic_, 10, std::bind(&ContactPointRecorder::onContactPoint, this, std::placeholders::_1));
        
        // 发布可视化标记
        if (enable_visualization_) {
            pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/contact_point_markers", 10);
        }
        
        // 定时器用于定期保存数据
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&ContactPointRecorder::saveData, this));
        
        // 初始化状态
        is_contacting_ = false;
        contact_start_time_ = 0.0;
        last_contact_time_ = 0.0;
        contact_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), 
            "ContactPointRecorder 初始化完成，输出目录: %s", output_directory_.c_str());
    }
    
    ~ContactPointRecorder() {
        // 保存最终数据
        saveData();
        savePointCloud();
        RCLCPP_INFO(this->get_logger(), "总共记录了 %d 个接触点", contact_count_);
    }

private:
    void onEePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_ee_pose_ = *msg;
        have_ee_pose_ = true;
    }
    
    void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if (msg->header.frame_id != "world") return;
        
        double force_magnitude = std::sqrt(
            msg->wrench.force.x * msg->wrench.force.x +
            msg->wrench.force.y * msg->wrench.force.y +
            msg->wrench.force.z * msg->wrench.force.z
        );
        
        double current_time = this->now().seconds();
        
        // 检测接触状态变化
        if (force_magnitude > force_threshold_) {
            if (!is_contacting_) {
                // 开始接触
                is_contacting_ = true;
                contact_start_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), "检测到接触开始，力大小: %.2f N", force_magnitude);
            }
            last_contact_time_ = current_time;
        } else {
            if (is_contacting_) {
                // 结束接触
                double contact_duration = current_time - contact_start_time_;
                if (contact_duration >= min_contact_duration_) {
                    // 记录有效的接触点
                    recordContactPoint(msg);
                }
                is_contacting_ = false;
                RCLCPP_INFO(this->get_logger(), "接触结束，持续时间: %.3f s", contact_duration);
            }
        }
    }
    
    void onContactPoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 直接记录来自控制器的测力点
        ContactPoint point;
        point.x = msg->pose.position.x;
        point.y = msg->pose.position.y;
        point.z = msg->pose.position.z;
        
        // 从四元数计算法向量（假设Z轴为法向量）
        double qw = msg->pose.orientation.w;
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        
        // 计算旋转矩阵的第三列（Z轴）
        point.nx = 2.0 * (qx * qz - qw * qy);
        point.ny = 2.0 * (qy * qz + qw * qx);
        point.nz = qw * qw - qx * qx - qy * qy + qz * qz;
        
        point.timestamp = this->now().seconds();
        point.frame_id = msg->header.frame_id;
        point.state_name = "CONTROLLER_CONTACT";
        
        // 获取当前力信息
        if (have_wrench_) {
            point.fx = current_force_x_;
            point.fy = current_force_y_;
            point.fz = current_force_z_;
        }
        
        contact_points_.push_back(point);
        contact_count_++;
        
        RCLCPP_INFO(this->get_logger(), 
            "记录测力点 #%d: (%.3f, %.3f, %.3f)", 
            contact_count_, point.x, point.y, point.z);
            
        // 发布可视化标记
        if (enable_visualization_) {
            publishContactPointMarker(point, contact_count_);
        }
    }
    
    void recordContactPoint(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench_msg) {
        if (!have_ee_pose_) return;
        
        ContactPoint point;
        point.x = current_ee_pose_.pose.position.x;
        point.y = current_ee_pose_.pose.position.y;
        point.z = current_ee_pose_.pose.position.z;
        
        // 计算法向量（从力向量推断）
        double force_magnitude = std::sqrt(
            wrench_msg->wrench.force.x * wrench_msg->wrench.force.x +
            wrench_msg->wrench.force.y * wrench_msg->wrench.force.y +
            wrench_msg->wrench.force.z * wrench_msg->wrench.force.z
        );
        
        if (force_magnitude > 1e-6) {
            point.nx = -wrench_msg->wrench.force.x / force_magnitude;
            point.ny = -wrench_msg->wrench.force.y / force_magnitude;
            point.nz = -wrench_msg->wrench.force.z / force_magnitude;
        } else {
            point.nx = 0.0;
            point.ny = 0.0;
            point.nz = 1.0;  // 默认Z轴向上
        }
        
        point.fx = wrench_msg->wrench.force.x;
        point.fy = wrench_msg->wrench.force.y;
        point.fz = wrench_msg->wrench.force.z;
        point.timestamp = this->now().seconds();
        point.frame_id = current_ee_pose_.header.frame_id;
        point.state_name = "FORCE_DETECTED";
        
        contact_points_.push_back(point);
        contact_count_++;
        
        RCLCPP_INFO(this->get_logger(), 
            "记录接触点 #%d: (%.3f, %.3f, %.3f), 力: (%.2f, %.2f, %.2f)", 
            contact_count_, point.x, point.y, point.z,
            point.fx, point.fy, point.fz);
            
        // 发布可视化标记
        if (enable_visualization_) {
            publishContactPointMarker(point, contact_count_);
        }
    }
    
    void publishContactPointMarker(const ContactPoint& point, int id) {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // 创建点标记
        auto point_marker = visualization_msgs::msg::Marker();
        point_marker.header.frame_id = "world";
        point_marker.header.stamp = this->now();
        point_marker.ns = "contact_points";
        point_marker.id = id;
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        
        point_marker.pose.position.x = point.x;
        point_marker.pose.position.y = point.y;
        point_marker.pose.position.z = point.z;
        point_marker.pose.orientation.w = 1.0;
        
        point_marker.scale.x = 0.01;  // 1cm半径
        point_marker.scale.y = 0.01;
        point_marker.scale.z = 0.01;
        
        // 根据力大小设置颜色
        double force_magnitude = std::sqrt(point.fx*point.fx + point.fy*point.fy + point.fz*point.fz);
        if (force_magnitude > 20.0) {
            point_marker.color.r = 1.0;  // 红色表示大力
            point_marker.color.g = 0.0;
            point_marker.color.b = 0.0;
        } else if (force_magnitude > 10.0) {
            point_marker.color.r = 1.0;  // 橙色表示中等力
            point_marker.color.g = 0.5;
            point_marker.color.b = 0.0;
        } else {
            point_marker.color.r = 0.0;  // 绿色表示小力
            point_marker.color.g = 1.0;
            point_marker.color.b = 0.0;
        }
        point_marker.color.a = 0.8;
        
        marker_array.markers.push_back(point_marker);
        
        // 创建法向量标记
        auto normal_marker = visualization_msgs::msg::Marker();
        normal_marker.header.frame_id = "world";
        normal_marker.header.stamp = this->now();
        normal_marker.ns = "contact_normals";
        normal_marker.id = id;
        normal_marker.type = visualization_msgs::msg::Marker::ARROW;
        normal_marker.action = visualization_msgs::msg::Marker::ADD;
        
        normal_marker.pose.position.x = point.x;
        normal_marker.pose.position.y = point.y;
        normal_marker.pose.position.z = point.z;
        
        // 计算法向量的方向
        double length = 0.02;  // 2cm长度
        normal_marker.pose.orientation.x = point.nx * length;
        normal_marker.pose.orientation.y = point.ny * length;
        normal_marker.pose.orientation.z = point.nz * length;
        normal_marker.pose.orientation.w = 1.0;
        
        normal_marker.scale.x = 0.005;  // 箭头粗细
        normal_marker.scale.y = 0.01;
        normal_marker.scale.z = length;
        
        normal_marker.color.r = 0.0;
        normal_marker.color.g = 0.0;
        normal_marker.color.b = 1.0;  // 蓝色表示法向量
        normal_marker.color.a = 0.8;
        
        marker_array.markers.push_back(normal_marker);
        
        pub_markers_->publish(marker_array);
    }
    
    void saveData() {
        if (contact_points_.empty()) return;
        
        // 保存为CSV格式
        std::string csv_filename = output_directory_ + "/contact_points.csv";
        std::ofstream csv_file(csv_filename);
        if (!csv_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建CSV文件: %s", csv_filename.c_str());
            return;
        }
        
        csv_file << "id,timestamp,x,y,z,nx,ny,nz,fx,fy,fz,frame_id,state_name\n";
        for (size_t i = 0; i < contact_points_.size(); ++i) {
            const auto& point = contact_points_[i];
            csv_file << i + 1 << ","
                     << point.timestamp << ","
                     << point.x << ","
                     << point.y << ","
                     << point.z << ","
                     << point.nx << ","
                     << point.ny << ","
                     << point.nz << ","
                     << point.fx << ","
                     << point.fy << ","
                     << point.fz << ","
                     << point.frame_id << ","
                     << point.state_name << "\n";
        }
        csv_file.close();
        
        RCLCPP_INFO(this->get_logger(), "已保存 %zu 个接触点到 %s", 
                    contact_points_.size(), csv_filename.c_str());
    }
    
    void savePointCloud() {
        if (contact_points_.empty()) return;
        
        // 保存为PLY格式（简单的ASCII格式）
        std::string ply_filename = output_directory_ + "/contact_points.ply";
        std::ofstream ply_file(ply_filename);
        if (!ply_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建PLY文件: %s", ply_filename.c_str());
            return;
        }
        
        // PLY文件头
        ply_file << "ply\n";
        ply_file << "format ascii 1.0\n";
        ply_file << "element vertex " << contact_points_.size() << "\n";
        ply_file << "property float x\n";
        ply_file << "property float y\n";
        ply_file << "property float z\n";
        ply_file << "property float nx\n";
        ply_file << "property float ny\n";
        ply_file << "property float nz\n";
        ply_file << "property float fx\n";
        ply_file << "property float fy\n";
        ply_file << "property float fz\n";
        ply_file << "end_header\n";
        
        // 写入点数据
        for (const auto& point : contact_points_) {
            ply_file << point.x << " " << point.y << " " << point.z << " "
                     << point.nx << " " << point.ny << " " << point.nz << " "
                     << point.fx << " " << point.fy << " " << point.fz << "\n";
        }
        ply_file.close();
        
        RCLCPP_INFO(this->get_logger(), "已保存点云到 %s", ply_filename.c_str());
    }
    
    void createOutputDirectory() {
        std::filesystem::path dir_path(output_directory_);
        if (!std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "创建输出目录: %s", output_directory_.c_str());
        }
    }
    
    // 成员变量
    std::string ee_pose_topic_, wrench_topic_, contact_points_topic_, output_directory_;
    double force_threshold_, min_contact_duration_;
    bool enable_visualization_;
    
    std::vector<ContactPoint> contact_points_;
    geometry_msgs::msg::PoseStamped current_ee_pose_;
    double current_force_x_, current_force_y_, current_force_z_;
    
    bool is_contacting_, have_ee_pose_, have_wrench_;
    double contact_start_time_, last_contact_time_;
    int contact_count_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_contact_points_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactPointRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 