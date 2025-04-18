#ifndef ROBOT_MONITOR_FACTORY_HPP_
#define ROBOT_MONITOR_FACTORY_HPP_

#include "communication/robot_monitor/robot_interface.hpp"
#include <string>
#include <map>
#include <memory>

namespace rclcpp
{
class Node;
}

namespace communication
{

/**
 * @brief Creates a robot monitor based on the robot type
 * 
 * @param robot_id Unique identifier for the robot
 * @param robot_type Type of robot ("husky" or "kobuki")
 * @param topics Map of topic names to topic paths
 * @param node Pointer to the ROS2 node
 * @return std::shared_ptr<RobotMonitor> Created robot monitor
 */
std::shared_ptr<RobotMonitor> createRobotMonitor(
    const std::string &robot_id,
        const std::string &robot_type,
        const std::map<std::string, std::string> &topics,
        std::shared_ptr<rclcpp::Node> node);

} // namespace communication

#endif // ROBOT_MONITOR_FACTORY_HPP_ 