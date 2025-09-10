#ifndef __LOCOBOT_ROBOT__HPP__
#define __LOCOBOT_ROBOT__HPP__

#include "communication/robot_monitor/robot_interface.hpp"
#include "communication/communicator.hpp"
#include "customed_interfaces/msg/locobot_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <irobot_create_msgs/msg/wheel_vels.hpp>

namespace communication
{

    class LocobotRobotMonitor : public RobotMonitor
    {
    public:
        explicit LocobotRobotMonitor(std::shared_ptr<rclcpp::Node> node, const std::string &robot_id)
            : RobotMonitor(node, robot_id, "locobot")
        {
            node_ = node;
            status_publisher_ = node_->create_publisher<customed_interfaces::msg::LocobotStatus>(
                "/locobotStatus", 10);
        }

        void initializeSubscribers(const std::map<std::string, std::string> &topics) override;

        // Callbacks for different status messages

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
        rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr wheel_vels_subscriber_;
        rclcpp::Publisher<customed_interfaces::msg::LocobotStatus>::SharedPtr status_publisher_;

        customed_interfaces::msg::LocobotStatus robot_status_;
        void fillPublisherMsg();
        bool hasJointStatesChanged(const sensor_msgs::msg::JointState &current, const sensor_msgs::msg::JointState &previous) const;
        void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
        void wheelVelsCallback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg);
        
    };

} // namespace communication

#endif // __LOCOBOT_ROBOT__HPP__