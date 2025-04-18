#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include "customed_interfaces/srv/request_history.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>
#include <filesystem>

class HistoryFileClient : public rclcpp::Node
{
public:
    HistoryFileClient() : Node("history_file_client")
    {
        // Create a service client
        client_ = this->create_client<customed_interfaces::srv::RequestHistory>("request_history");

        // Create a subscription to the topic where the file will be published
        subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "/history_log_file", 10,
            std::bind(&HistoryFileClient::callback, this, std::placeholders::_1));

        // Call the service to request the file
        callService();
    }

private:
    rclcpp::Client<customed_interfaces::srv::RequestHistory>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
    std::string received_file_path_;

    void callService()
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        auto request = std::make_shared<customed_interfaces::srv::RequestHistory::Request>();

        auto future_result = client_->async_send_request(
            request,
            [this](rclcpp::Client<customed_interfaces::srv::RequestHistory>::SharedFuture response)
            {
                // Handle the service response
                try
                {
                    auto result = response.get();
                    if (result->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Service call failed");
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed with exception: %s", e.what());
                }
            });
    }

    void callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {

        // auto package_path = ament_index_cpp::get_package_share_directory("communication");
        // received_file_path_ = package_path + "/received_history_log.json";
        // received_file_path_ = "/home/maya/workspaces/Digital_Twin/received_history_log.json";

        auto source_path = std::filesystem::path(__FILE__);
        auto workspace_src = source_path.parent_path().parent_path().parent_path(); // Gets you to .../src/communication
        auto history_dir = workspace_src / "communication" / "history";             // .../src/communication/history
        received_file_path_ = (history_dir / ("received_history_log.json")).string();

        std::ofstream file(received_file_path_, std::ios::binary);

        if (!file)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file for writing: %s", received_file_path_.c_str());
            return;
        }

        // Write the received data to the file
        file.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());

        if (file.good())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully wrote %ld bytes to %s.", msg->data.size(), received_file_path_.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write data to the file.");
        }

        file.close();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HistoryFileClient>());
    rclcpp::shutdown();
    return 0;
}
