#include "communication/robot_monitor/robot_interface.hpp"
#include "communication/robot_monitor/husky_robot.hpp"
#include "communication/robot_monitor/kobuki_robot.hpp"
#include <memory>

namespace communication
{

    std::shared_ptr<RobotMonitor> createRobotMonitor(
        const std::string &robot_id,
        const std::string &robot_type,
        const std::map<std::string, std::string> &topics,
        std::shared_ptr<rclcpp::Node> node)
    {

        std::shared_ptr<RobotMonitor> monitor;

        if (robot_type == "Husky")
        {
            monitor = std::make_shared<HuskyRobotMonitor>(node, robot_id);
        }
        else if (robot_type == "Kobuki")
        {
            monitor = std::make_shared<KobukiRobotMonitor>(node, robot_id);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("robot_monitor_factory"),
                         "Unknown robot type: %s", robot_type.c_str());
            return nullptr;
        }

        if (!topics.empty())
        {
            monitor->initializeSubscribers(topics);
        }
        return monitor;
    }

} // namespace communication