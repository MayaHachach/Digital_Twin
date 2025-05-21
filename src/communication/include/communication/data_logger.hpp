#ifndef __DATA__LOGGER__HPP__
#define __DATA__LOGGER__HPP__

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "customed_interfaces/msg/object.hpp"
#include "customed_interfaces/msg/husky_status.hpp"
#include "customed_interfaces/msg/kobuki_status.hpp"
#include "communication/robot_monitor/robot_interface.hpp"
#include <unordered_map>
#include <fstream>
#include <string>
#include <filesystem>

#include <thread>
#include <queue>
#include <mutex>
#include <atomic>

using namespace std;

class DataLogger
{
private:
    std::mutex log_mutex_;
    std::queue<nlohmann::json> log_queue_;
    std::atomic<bool> keep_logging_;
    std::thread logging_thread_;

public:
    std::string log_file_path_;

    struct StatusUnion
    {
        std::string type; // e.g., "Husky", "Kobuki", or "None"

        customed_interfaces::msg::HuskyStatus husky_status;
        customed_interfaces::msg::KobukiStatus kobuki_status;

        StatusUnion() : type("None") {}
    };

    struct object_map_struct
    {
        string topic_name;
        customed_interfaces::msg::Object message;
    };

    unordered_map<string, vector<object_map_struct>> object_map;
    unordered_map<string, vector<customed_interfaces::msg::Object>> temp_map;

    // DataLogger();
    explicit DataLogger(const std::string &user_file_name);
    void initializeJsonFile();
    void logData(const nlohmann::json &new_entry);
    std::string GetTimestamp();
    nlohmann::json ROSMessageToJson(const customed_interfaces::msg::Object &msg, const std::string &timestamp);
    geometry_msgs::msg::Pose getLastPose(const std::string &name, int id);
    unordered_map<string, vector<object_map_struct>> loadLastIterationToMap();
    unordered_map<string, vector<customed_interfaces::msg::Object>> loadLastTempToMap();
    void logAllObjects(std::unordered_map<std::string, vector<object_map_struct>> &objectMap);
    void logAllObjects(std::unordered_map<std::string, vector<DataLogger::object_map_struct>> &objectMap, std::map<std::string, std::shared_ptr<communication::RobotMonitor>> &robotMonitors);
    void logTempObjects(std::unordered_map<std::string, vector<customed_interfaces::msg::Object>> &Map);
    std::string removeIDFromName(const std::string &name, int id);

    void startLoggingThread();
    void stopLoggingThread();

};

#endif //__DATA__LOGGER__HPP__
