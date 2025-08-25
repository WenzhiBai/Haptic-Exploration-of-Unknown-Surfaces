#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // 订阅的原始关节状态消息
#include <mujoco_ros2_control_demos/msg/joint_state_array.hpp> // 自定义整合后的消息
    
#include <iostream>
#include <map>       
#include <string>     
#include <vector>     
#include <mutex>      // 用于线程安全访问共享数据

class StateManagerNode : public rclcpp::Node
{
public:
    StateManagerNode() : Node("state_manager_node")
    {

        // 1. 创建 /joint_states 话题的订阅器
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(), // 使用 SensorDataQoS 以获得最新的数据
            std::bind(&StateManagerNode::jointStateCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "==============================================================");
        RCLCPP_INFO(this->get_logger(), "StateManagerNode: Subscribing to /joint_states...");

        // 2. 创建自定义话题的发布器，用于发布整合后的信息
        // 话题名称可以自定义，这里我用了 /robot_state_for_controller
        integrated_state_publisher_ = this->create_publisher<mujoco_ros2_control_demos::msg::JointStateArray>(
            "/robot_state_for_controller",
            10 // QoS history depth
        );
        RCLCPP_INFO(this->get_logger(), "StateManagerNode: Publishing to /robot_state_for_controller...");
        RCLCPP_INFO(this->get_logger(), "==============================================================");

        // 初始化关节名称列表 (可选，但有助于验证顺序和处理首次接收)
        // 在实际应用中，你可能从URDF或参数服务中加载这些名称
        // 这里假设我们知道FR3的关节名称
        expected_joint_names_ = {
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7"
        };
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 使用互斥锁保护共享数据 (maps) 的访问，防止多线程竞争
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 遍历接收到的关节状态消息，并将其存入 map
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint_name = msg->name[i];

            // 存储位置
            if (i < msg->position.size()) {
                joint_positions_[joint_name] = msg->position[i];
            } else {
                joint_positions_[joint_name] = 0.0; // 如果数据缺失，给个默认值
            }
            // 存储速度
            if (i < msg->velocity.size()) {
                joint_velocities_[joint_name] = msg->velocity[i];
            } else {
                joint_velocities_[joint_name] = 0.0;
            }
            // 存储力矩
            if (i < msg->effort.size()) {
                joint_efforts_[joint_name] = msg->effort[i];
            } else {
                joint_efforts_[joint_name] = 0.0;
            }
        }

        // 整合数据并发布
        publishIntegratedState(msg->header);
    }

    void publishIntegratedState(const std_msgs::msg::Header& header)
    {
        auto integrated_msg = std::make_unique<mujoco_ros2_control_demos::msg::JointStateArray>();
        integrated_msg->header = header; // 沿用原始时间戳和 frame_id

        // 按照预定义的顺序填充消息，可以保证控制器接收到的数据顺序一致
        for (const std::string& name : expected_joint_names_) {
            integrated_msg->name.push_back(name);
            integrated_msg->position.push_back(joint_positions_[name]);
            integrated_msg->velocity.push_back(joint_velocities_[name]);
            integrated_msg->effort.push_back(joint_efforts_[name]);
        }
        
        integrated_state_publisher_->publish(std::move(integrated_msg));
        // RCLCPP_INFO(this->get_logger(), "StateManagerNode: Published integrated joint states.");
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<mujoco_ros2_control_demos::msg::JointStateArray>::SharedPtr integrated_state_publisher_;

    // 使用 map 存储关节状态，方便按名称查找
    std::map<std::string, double> joint_positions_;
    std::map<std::string, double> joint_velocities_;
    std::map<std::string, double> joint_efforts_;
    std::mutex data_mutex_; // 保护 maps 的访问

    std::vector<std::string> expected_joint_names_; // 预期的关节名称列表，用于保证发布顺序
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateManagerNode>());
    rclcpp::shutdown();
    return 0;
}