#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>
#include "customed_interfaces/srv/request_history.hpp"
#include "customed_interfaces/msg/object.hpp"
#include "communication/data_logger.hpp"

class STODServer : public rclcpp::Node
{
public:
    STODServer() : Node("STOD_Server"), logger("initial_history")
    {
        STOD_service = this->create_service<customed_interfaces::srv::RequestHistory>(
            "request_STOD",
            std::bind(&STODServer::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

        STOD_publisher = this->create_publisher<customed_interfaces::msg::Object>("/STOD", 10);
        current_STOD = logger.loadLastIterationToMap();
    }

private:
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_publisher;
    rclcpp::Service<customed_interfaces::srv::RequestHistory>::SharedPtr STOD_service;
    std::unordered_map<std::string, vector<DataLogger::object_map_struct>> current_STOD;
    DataLogger logger;

    void handleRequest(
        const std::shared_ptr<customed_interfaces::srv::RequestHistory::Request> request_,
        std::shared_ptr<customed_interfaces::srv::RequestHistory::Response> response_)
    {
        RCLCPP_INFO(this->get_logger(), "Current Object Map:");
        for (const auto &pair : current_STOD)
        {
            RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
            for (const auto &obj : pair.second)
            {
                RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                            obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
                STOD_publisher->publish(obj.message);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /STOD", current_STOD.size());
        response_->success = true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STODServer>());
    rclcpp::shutdown();
    return 0;
}