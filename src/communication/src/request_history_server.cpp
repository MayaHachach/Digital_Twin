#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>
#include "customed_interfaces/srv/request_history.hpp"

class HistoryFileServer : public rclcpp::Node {
public:
    HistoryFileServer() : Node("History_file_Server") {

        service_ = this->create_service<customed_interfaces::srv::RequestHistory>(
            "request_history",
            std::bind(&HistoryFileServer::handleRequest, this, std::placeholders::_1, std::placeholders::_2)
        );

        
        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/history_log_file", 10);

    }

private:
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Service<customed_interfaces::srv::RequestHistory>::SharedPtr service_;

    void handleRequest(
        const std::shared_ptr<customed_interfaces::srv::RequestHistory::Request> request,
        std::shared_ptr<customed_interfaces::srv::RequestHistory::Response> response)
        {

            auto package_path = ament_index_cpp::get_package_share_directory("communication");
            std::string file_path = package_path + "/initial_history.json";

            std::ifstream file(file_path, std::ios::binary);
            if (!file) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
                response->success = false;
                return;
            }

            // Read the file into a buffer
            std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(file), {});
            auto msg = std_msgs::msg::ByteMultiArray();
            msg.data = buffer;

            // Publish the message
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published file: %s", file_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Published %ld bytes to /history_log_file", buffer.size());
            response -> success = true;
            
        }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HistoryFileServer>());
    rclcpp::shutdown();
    return 0;
}