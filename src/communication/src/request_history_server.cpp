#include "communication/request_history_server.hpp"
#include <filesystem>

HistoryFileServer::HistoryFileServer() : Node("History_file_Server")
{

    service_ = this->create_service<customed_interfaces::srv::RequestHistory>(
        "request_history",
        std::bind(&HistoryFileServer::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/history_log_file", 10);
}

void HistoryFileServer::handleRequest(
    const std::shared_ptr<customed_interfaces::srv::RequestHistory::Request> request,
    std::shared_ptr<customed_interfaces::srv::RequestHistory::Response> response)
{
    auto source_path = std::filesystem::path(__FILE__);
    auto workspace_src = source_path.parent_path().parent_path().parent_path(); // Gets you to .../src/communication
    auto history_dir = workspace_src / "communication" / "history";             // .../src/communication/history
    std::string file_path = (history_dir / ("initial_history.json")).string();

    std::ifstream file(file_path, std::ios::binary);
    if (!file)
    {
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
    response->success = true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HistoryFileServer>());
    rclcpp::shutdown();
    return 0;
}