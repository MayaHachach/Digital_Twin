#ifndef __DATA__LOGGER__HPP__
#define __DATA__LOGGER__HPP__

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "customed_interfaces/msg/object.hpp"
#include <unordered_map>
#include <fstream>
#include <string>

using namespace std;

class DataLogger
{
private:
public:
    std::string log_file_path_;

    struct object_map_struct
    {
        // int id;
        string topic_name;
        customed_interfaces::msg::Object message;
    };

    unordered_map<string, vector<object_map_struct>> object_map;
    unordered_map<std::string, vector<customed_interfaces::msg::Object>> temp_map;

    // DataLogger();
    explicit DataLogger(const std::string &user_file_name);
    void initializeJsonFile();
    void logData(const nlohmann::json &new_entry);
    std::string GetTimestamp();
    nlohmann::json ROSMessageToJson(const customed_interfaces::msg::Object &msg, const std::string &timestamp);
    unordered_map<string, vector<object_map_struct>> loadLastIterationToMap();
    unordered_map<string, vector<customed_interfaces::msg::Object>> loadLastTempToMap();
    void logAllObjects(std::unordered_map<std::string, vector<object_map_struct>> &objectMap);
    void logTempObjects(std::unordered_map<std::string, vector<customed_interfaces::msg::Object>> &Map);
    std::string removeIDFromName(const std::string &name, int id);
};

#endif //__DATA__LOGGER__HPP__
