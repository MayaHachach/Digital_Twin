#include "communication/robot_monitor/kobuki_robot.hpp"
#include <rclcpp/rclcpp.hpp>

namespace communication
{

    // Most of the implementation is in the header file since we're using templates
    // and the callbacks are defined inline. The constructor and initializeSubscribers
    // are already implemented in the header.

    void KobukiRobotMonitor::initializeSubscribers(const std::map<std::string, std::string> &topics)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        try
        {
            RCLCPP_INFO(this->node_->get_logger(), "Initializing subscribers...");
            for (const auto& topic : topics)
            {
                RCLCPP_INFO(this->node_->get_logger(), "Found topic: %s", topic.first.c_str());
            }
            // Initialize battery subscriber
            auto battery_it = topics.find("battery_state");
            if (battery_it != topics.end())
            {
                battery_subscriber_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
                    battery_it->second, qos_profile,
                    std::bind(&KobukiRobotMonitor::batteryCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->node_->get_logger(), "Created Kobuki battery subscriber for topic: %s",
                            battery_it->second.c_str());
            }

            // Initialize wheel status subscriber
            auto wheel_status_it = topics.find("wheel_status");
            if (wheel_status_it != topics.end())
            {
                wheel_status_subscriber_ = this->node_->create_subscription<irobot_create_msgs::msg::WheelStatus>(
                    wheel_status_it->second, qos_profile,
                    std::bind(&KobukiRobotMonitor::wheelStatusCallback, this, std::placeholders::_1));
                RCLCPP_INFO(this->node_->get_logger(), "Created Kobuki wheel status subscriber for topic: %s",
                            wheel_status_it->second.c_str());
            }

            // Initialize wheel velocities subscriber
            auto wheel_vels_it = topics.find("wheel_vel");
            if (wheel_vels_it != topics.end())
            {
                wheel_vels_subscriber_ = this->node_->create_subscription<irobot_create_msgs::msg::WheelVels>(wheel_vels_it->second, qos_profile,
                                                                                                              std::bind(&KobukiRobotMonitor::wheelVelsCallback, this, std::placeholders::_1));

                RCLCPP_INFO(this->node_->get_logger(), "Created Kobuki wheel velocities subscriber for topic: %s", wheel_vels_it->second.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->node_->get_logger(),
                         "Failed to initialize Kobuki subscribers: %s", e.what());
        }
    }

    void KobukiRobotMonitor::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        status_.data["battery_percentage"] = msg->percentage; // Convert to percentage
        status_.data["battery_voltage"] = msg->voltage;
        status_.data["battery_current"] = msg->current;
        status_.data["battery_temperature"] = msg->temperature;

        // RCLCPP_INFO(this->node_->get_logger(),
        //              "Updated battery status for Kobuki robot %s: %.2f%%",
        //              robot_id_.c_str(), status_.data["battery_percentage"]);

        // Fill the custom message with the status data
        fillPublisherMsg();
    }

    void KobukiRobotMonitor::wheelStatusCallback(const irobot_create_msgs::msg::WheelStatus::SharedPtr msg)
    {
        status_.data["left_driver_current"] = msg->current_ma_left / 1000.0;   // Convert mA to A
        status_.data["right_driver_current"] = msg->current_ma_right / 1000.0; // Convert mA to A
        status_.data["pwm_left"] = msg->pwm_left;
        status_.data["pwm_right"] = msg->pwm_right;
        status_.data["wheels_enabled"] = msg->wheels_enabled? 1.0 : 0.0;

        // RCLCPP_INFO(this->node_->get_logger(),
        //              "Updated wheel status for Kobuki robot %s", robot_id_.c_str());

        // Fill the custom message with the status data
        fillPublisherMsg();
    }

    void KobukiRobotMonitor::wheelVelsCallback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg)
    {
        // Calculate linear and angular velocity from wheel velocities
        double wheel_separation = 0.23; // Distance between wheels in meters
        status_.data["linear_velocity"] = (msg->velocity_left + msg->velocity_right) / 2.0;
        status_.data["angular_velocity"] = (msg->velocity_right - msg->velocity_left) / wheel_separation;

        status_.data["left_wheel_velocity"] = msg->velocity_left;
        status_.data["right_wheel_velocity"] = msg->velocity_right;

        // RCLCPP_INFO(this->node_->get_logger(),
        //              "Updated wheel velocities for Kobuki robot %s: v_linear=%.2f, v_angular=%.2f",
        //              robot_id_.c_str(), status_.data["linear_velocity"], status_.data["angular_velocity"]);

        // Fill the custom message with the status data
        fillPublisherMsg();
    }

    void KobukiRobotMonitor::fillPublisherMsg()
    {
        // Fill the custom message with the status data
        robot_status_.name = robot_id_;
        robot_status_.percentage = status_.data["battery_percentage"];
        robot_status_.voltage = status_.data["battery_voltage"];
        robot_status_.current = status_.data["battery_current"];
        robot_status_.temperature = status_.data["battery_temperature"];
        robot_status_.current_ma_left = status_.data["left_driver_current"];
        robot_status_.current_ma_right = status_.data["right_driver_current"];
        robot_status_.pwm_left = status_.data["pwm_left"];
        robot_status_.pwm_right = status_.data["pwm_right"];
        robot_status_.wheels_enabled = status_.data["wheels_enabled"] == 1;
        robot_status_.linear_velocity = status_.data["linear_velocity"];
        robot_status_.angular_velocity = status_.data["angular_velocity"];
        robot_status_.velocity_left = status_.data["left_wheel_velocity"];
        robot_status_.velocity_right = status_.data["right_wheel_velocity"];

        // Publish the custom message
        status_publisher_->publish(robot_status_);
        // RCLCPP_INFO(this->node_->get_logger(),
        //              "Published Kobuki status for robot %s", robot_id_.c_str());

    }

} // namespace communication