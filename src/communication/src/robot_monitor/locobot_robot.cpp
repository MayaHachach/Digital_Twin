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

            // Initialize battery subscriber
            auto battery_it = topics.find("battery_state");
            if (battery_it != topics.end())
            {
                battery_subscriber_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
                    battery_it->second, qos_profile,
                    std::bind(&LocobotRobotMonitor::batteryCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->node_->get_logger(), "Created locobot battery subscriber for topic: %s",
                            battery_it->second.c_str());
            }

            // Initialize wheel velocities subscriber
            auto wheel_vels_it = topics.find("wheel_vel");
            if (wheel_vels_it != topics.end())
            {
                wheel_vels_subscriber_ = this->node_->create_subscription<irobot_create_msgs::msg::WheelVels>(wheel_vels_it->second, qos_profile,
                                                                                                              std::bind(&LocobotRobotMonitor::wheelVelsCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->node_->get_logger(), "Created locobot wheel velocities subscriber for topic: %s", wheel_vels_it->second.c_str());
            }


        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->node_->get_logger(),
                         "Failed to initialize locobot subscribers: %s", e.what());
        }
    }

    void LocobotRobotMonitor::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        status_.data["battery_percentage"] = msg->percentage; // Convert to percentage
        status_.data["battery_voltage"] = msg->voltage;
        status_.data["battery_current"] = msg->current;
        status_.data["battery_temperature"] = msg->temperature;

        // Fill the custom message with the status data
        fillPublisherMsg();
    }

    void LocobotRobotMonitor::wheelVelsCallback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg)
    {
        // Calculate linear and angular velocity from wheel velocities
        double wheel_separation = 0.23; // Distance between wheels in meters
        status_.data["linear_velocity"] = (msg->velocity_left + msg->velocity_right) / 2.0;
        status_.data["angular_velocity"] = (msg->velocity_right - msg->velocity_left) / wheel_separation;

        status_.data["left_wheel_velocity"] = msg->velocity_left;
        status_.data["right_wheel_velocity"] = msg->velocity_right;

        // Fill the custom message with the status data
        fillPublisherMsg();
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
        robot_status_.percentage = status_.data["battery_percentage"];
        robot_status_.voltage = status_.data["battery_voltage"];
        robot_status_.current = status_.data["battery_current"];        // Publish the custom message
        robot_status_.linear_velocity = status_.data["linear_velocity"];
        robot_status_.angular_velocity = status_.data["angular_velocity"];
        robot_status_.velocity_left = status_.data["left_wheel_velocity"];
        robot_status_.velocity_right = status_.data["right_wheel_velocity"];
        robot_status_.temperature = status_.data["battery_temperature"];
        status_publisher_->publish(robot_status_);
    }

} // namespace communication