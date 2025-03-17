#ifndef __REQUEST__STOD__SERVER__
#define __REQUEST__STOD__SERVER__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>
#include "customed_interfaces/srv/request_stod.hpp"
#include "customed_interfaces/msg/object.hpp"
#include "communication/data_logger.hpp"
#include <mutex>

using namespace std;
class STODServer : public rclcpp::Node
{
private:
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_publisher;
    rclcpp::Service<customed_interfaces::srv::RequestSTOD>::SharedPtr STOD_service;
    std::unordered_map<std::string, std::vector<DataLogger::object_map_struct>> current_STOD;
    DataLogger logger;
    std::mutex request_mutex_;
    bool request_processed_;

    void handleRequest(const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
                       std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_);

public:
    STODServer(/* args */);
    void publishSTOD();
};

#endif