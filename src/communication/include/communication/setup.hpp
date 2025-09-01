#ifndef __SETUP__HPP__
#define __SETUP__HPP__

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include "communication/data_logger.hpp"
#include <customed_interfaces/msg/object.hpp>
#include "communication/robot_monitor/robot_interface.hpp"
#include "yaml-cpp/yaml.h"
#include <filesystem> // C++17
#include <fstream>
#include <algorithm>

class SetupNode : public rclcpp::Node
{
    public:
    SetupNode();
    void STODExtractionCallback(const customed_interfaces::msg::Object::SharedPtr msg);
    void GenerateConfigFile();
    bool isObjectInSTOD(const std::string &object_class_name, int object_id = 0);
    DataLogger::object_map_struct AddNewObject(const customed_interfaces::msg::Object &message);
    
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr initial_stod_subscriber_;
    DataLogger logger_;
    std::unordered_map<std::string, std::vector<DataLogger::object_map_struct>> object_map_;
    std::unordered_map<std::string, int> object_counts_;
    int total_object_count_;
    
};


#endif // __SETUP__HPP__