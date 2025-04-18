#ifndef __HUSKY_ROBOT__HPP__
#define __HUSKY_ROBOT__HPP__

#include "communication/robot_monitor/robot_interface.hpp"
#include "customed_interfaces/msg/husky_status.hpp"
#include "husky_msgs/msg/husky_status.hpp"
#include "communication/communicator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace communication
{

    class HuskyRobotMonitor : public RobotMonitor
    {
    public:
        explicit HuskyRobotMonitor(std::shared_ptr<rclcpp::Node> node, const std::string &robot_id)
            : RobotMonitor(node, robot_id, "husky")
        {
            node_ = node;
            status_publisher_ = node_->create_publisher<customed_interfaces::msg::HuskyStatus>(
                "/huskyStatus", 10);
        }

        void initializeSubscribers(const std::map<std::string, std::string> &topics) override;

        // Callback for status messages
        void statusCallback(const husky_msgs::msg::HuskyStatus::SharedPtr msg);

    private:
        // node handle
        std::shared_ptr<rclcpp::Node> node_;

        rclcpp::Subscription<husky_msgs::msg::HuskyStatus>::SharedPtr status_subscriber_;
        rclcpp::Publisher<customed_interfaces::msg::HuskyStatus>::SharedPtr status_publisher_;

        customed_interfaces::msg::HuskyStatus robot_status_;
        float min_voltage = 16.0, max_voltage = 26.4;

        void fillPublisherMsg();

    };

} // namespace communication

#endif // __HUSKY_ROBOT__HPP__