#ifndef __KOBUKI_ROBOT__HPP__
#define __KOBUKI_ROBOT__HPP__

#include "communication/robot_monitor/robot_interface.hpp"
#include "communication/communicator.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <irobot_create_msgs/msg/wheel_status.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace communication
{

    class KobukiRobotMonitor : public RobotMonitor
    {
    public:
        explicit KobukiRobotMonitor(std::shared_ptr<rclcpp::Node> node, const std::string &robot_id)
            : RobotMonitor(node, robot_id, "kobuki")
        {
            node_ = node;
            status_publisher_ = node_->create_publisher<customed_interfaces::msg::KobukiStatus>(
                "/kobukiStatus", 10);
        }

        void initializeSubscribers(const std::map<std::string, std::string> &topics) override;

        // Callbacks for different status messages
        void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
        void wheelStatusCallback(const irobot_create_msgs::msg::WheelStatus::SharedPtr msg);
        void wheelVelsCallback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg);
        // void navigationPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
        rclcpp::Subscription<irobot_create_msgs::msg::WheelStatus>::SharedPtr wheel_status_subscriber_;
        rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr wheel_vels_subscriber_;
        // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
        rclcpp::Publisher<customed_interfaces::msg::KobukiStatus>::SharedPtr status_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr navigation_path_subscriber_;
        
        customed_interfaces::msg::KobukiStatus robot_status_;
        void fillPublisherMsg();
    };

} // namespace communication

#endif // __KOBUKI_ROBOT__HPP__