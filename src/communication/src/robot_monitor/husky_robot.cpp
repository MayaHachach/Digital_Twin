#include "communication/robot_monitor/husky_robot.hpp"
#include <rclcpp/rclcpp.hpp>

namespace communication
{

    // Most of the implementation is in the header file since we're using templates
    // and the callbacks are defined inline. The constructor and initializeSubscribers
    // are already implemented in the header.

    void HuskyRobotMonitor::initializeSubscribers(const std::map<std::string, std::string> &topics)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        auto it = topics.find("robot_status");
        if (it != topics.end())
        {
            try
            {
                // Initialize Husky status subscriber
                status_subscriber_ = node_->create_subscription<husky_msgs::msg::HuskyStatus>(it->second, qos_profile,
                                                                                                      std::bind(&HuskyRobotMonitor::statusCallback, this, std::placeholders::_1));
                RCLCPP_INFO(node_->get_logger(), "Created Husky status subscriber for topic: %s", it->second.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->node_->get_logger(),
                             "Failed to create Husky status subscriber for topic %s: %s",
                             it->second.c_str(), e.what());
            }
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(),
                         "No status topic found for Husky robot %s", robot_id_.c_str());
        }
    }

    void HuskyRobotMonitor::statusCallback(const husky_msgs::msg::HuskyStatus::SharedPtr msg)
    {
        auto battery_voltage = msg->battery_voltage;
        auto battery_percentage = ((battery_voltage - min_voltage)/(max_voltage - min_voltage))*100;
        
        // Store additional data specific to Husky
        status_.data["battery_percentage"] = battery_percentage;
        status_.data["battery_voltage"] = msg->battery_voltage;
        status_.data["left_driver_current"] = msg->left_driver_current;
        status_.data["right_driver_current"] = msg->right_driver_current;
        status_.data["left_driver_voltage"] = msg->left_driver_voltage;
        status_.data["right_driver_voltage"] = msg->right_driver_voltage;
        status_.data["left_driver_temp"] = msg->left_driver_temp;
        status_.data["right_driver_temp"] = msg->right_driver_temp;
        status_.data["left_motor_temp"] = msg->left_motor_temp;
        status_.data["right_motor_temp"] = msg->right_motor_temp;

        // Store error conditions
        status_.data["timeout"] = msg->timeout ? 1.0 : 0.0;
        status_.data["lockout"] = msg->lockout ? 1.0 : 0.0;
        status_.data["e_stop"] = msg->e_stop ? 1.0 : 0.0;
        status_.data["ros_pause"] = msg->ros_pause ? 1.0 : 0.0;
        status_.data["no_battery"] = msg->no_battery ? 1.0 : 0.0;
        status_.data["current_limit"] = msg->current_limit ? 1.0 : 0.0;

        // RCLCPP_INFO(this->node_->get_logger(),
        //              "Updated status for Husky robot %s: battery=%.2f%%",
        //              robot_id_.c_str(), status_.data["battery_percentage"]);

        // Fill the custom message with the status data
        fillPublisherMsg();
        // Publish the custom message
        status_publisher_->publish(robot_status_);
    }

    void HuskyRobotMonitor::fillPublisherMsg()
    {
        // Fill the custom message with the status data
        robot_status_.name = robot_id_;
        robot_status_.battery_percentage = status_.data["battery_percentage"];
        robot_status_.battery_voltage = status_.data["battery_voltage"];
        robot_status_.left_driver_current = status_.data["left_driver_current"];
        robot_status_.right_driver_current = status_.data["right_driver_current"];
        robot_status_.left_driver_voltage = status_.data["left_driver_voltage"];
        robot_status_.right_driver_voltage = status_.data["right_driver_voltage"];
        robot_status_.left_driver_temp = status_.data["left_driver_temp"];
        robot_status_.right_driver_temp = status_.data["right_driver_temp"];
        robot_status_.left_motor_temp = status_.data["left_motor_temp"];
        robot_status_.right_motor_temp = status_.data["right_motor_temp"];
        robot_status_.timeout = status_.data["timeout"] == 1.0;
        robot_status_.lockout = status_.data["lockout"] == 1.0;
        robot_status_.e_stop = status_.data["e_stop"] == 1.0;
        robot_status_.ros_pause = status_.data["ros_pause"] == 1.0;
        robot_status_.no_battery = status_.data["no_battery"] == 1.0;
        robot_status_.current_limit = status_.data["current_limit"] == 1.0;
    }

} // namespace communication