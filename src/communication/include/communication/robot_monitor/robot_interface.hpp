#ifndef ROBOT_INTERFACE_HPP__
#define ROBOT_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <memory>

namespace communication
{
    struct RobotStatus
    {
        std::string robot_id;
        std::string robot_type;
        std::map<std::string, double> data; // For any extra data specific to robot type
        nav_msgs::msg::Path navigation_path_data;
        sensor_msgs::msg::JointState joint_states_data;

        // default constructor
        RobotStatus(){}
    };

    class RobotMonitor
    {
    public:
        explicit RobotMonitor(std::shared_ptr<rclcpp::Node> node, const std::string &robot_id, const std::string &robot_type)
            : robot_id_(robot_id), robot_type_(robot_type)
        {
            status_.robot_id = robot_id;
            status_.robot_type = robot_type;
        }

        virtual ~RobotMonitor() = default;

        //Pure virtual function to initialize subscribers
        virtual void initializeSubscribers(const std::map<std::string, std::string> &topics) = 0;

        //set the navigation path
        void setNavigationPath(const nav_msgs::msg::Path &path)
        {
            status_.navigation_path_data = path;
            RCLCPP_INFO(rclcpp::get_logger("RobotMonitor"), "Set navigation path for robot %s", robot_id_.c_str());

        }

        //Get the current status
        const RobotStatus &getStatus() const
        {
            return status_;
        }

    protected:
        RobotStatus status_;
        std::string robot_id_;
        std::string robot_type_;
    };

} //namespace communication

#endif // ROBOT_INTERFACE_HPP__