#include "communication/robot_monitor/locobot_robot.hpp"
#include <rclcpp/rclcpp.hpp>

namespace communication
{

    // Most of the implementation is in the header file since we're using templates
    // and the callbacks are defined inline. The constructor and initializeSubscribers
    // are already implemented in the header.

    void LocobotRobotMonitor::initializeSubscribers(const std::map<std::string, std::string> &topics)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        try
        {
            RCLCPP_INFO(this->node_->get_logger(), "Initializing subscribers...");

            for (const auto &topic : topics)
            {
                RCLCPP_INFO(this->node_->get_logger(), "Found topic: %s", topic.first.c_str());
            }

            auto joint_states_it = topics.find("joint_states");
            if (joint_states_it != topics.end())
            {
                RCLCPP_WARN(this->node_->get_logger(), "Joint states topic found: %s", joint_states_it->second.c_str());
                
                joint_states_subscriber_ = this->node_->create_subscription<sensor_msgs::msg::JointState>(
                    joint_states_it->second, qos_profile,
                    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                    {
                        // Handle joint states if needed
                        RCLCPP_INFO(this->node_->get_logger(), "Received joint states for Locobot robot %s", robot_id_.c_str());
                        if (hasJointStatesChanged(*msg, status_.joint_states_data))
                        {
                            RCLCPP_INFO(this->node_->get_logger(), "Joint states have changed for Locobot robot %s", robot_id_.c_str());
                            status_.joint_states_data = *msg;
                            fillPublisherMsg();

                            if (joint_state_change_callback_)
                                joint_state_change_callback_();
                        }
                        // status_.joint_states_data = *msg;
                        // RCLCPP_INFO(this->node_->get_logger(),
                        //             "Joint states: %s", msg->name.size() > 0 ? msg->name[0].c_str() : "No joints");

                        // Fill the custom message with the status data
                        // fillPublisherMsg();
                    });
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->node_->get_logger(),
                         "Failed to initialize locobot subscribers: %s", e.what());
        }
    }

    bool LocobotRobotMonitor::hasJointStatesChanged(const sensor_msgs::msg::JointState &current, const sensor_msgs::msg::JointState &previous) const
    {
        if (current.name != previous.name ||
            current.position != previous.position ||
            current.velocity != previous.velocity ||
            current.effort != previous.effort)
        {
            return true;
        }
        return false;
    }

    void LocobotRobotMonitor::fillPublisherMsg()
    {
        // Fill the custom message with the status data
        robot_status_.name = robot_id_;
        robot_status_.joint_states = status_.joint_states_data;

        // Publish the custom message
        status_publisher_->publish(robot_status_);
    }

} // namespace communication