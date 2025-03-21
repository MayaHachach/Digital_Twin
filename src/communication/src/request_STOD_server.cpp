#include "communication/request_STOD_server.hpp"

STODServer::STODServer() : Node("STOD_Server"), logger("initial_history"), request_processed_(false)
{
    STOD_service = this->create_service<customed_interfaces::srv::RequestSTOD>(
        "request_STOD",
        std::bind(&STODServer::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

    STOD_publisher = this->create_publisher<customed_interfaces::msg::Object>("/STOD", 10);
    current_STOD = logger.loadLastIterationToMap();
}

void STODServer::publishSTOD()
{
    RCLCPP_INFO(this->get_logger(), "Processing request and publishing objects...");
    for (const auto &pair : current_STOD)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
            STOD_publisher->publish(obj.message);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /STOD", current_STOD.size());
}

void STODServer::handleRequest(
    const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
    std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_)
{
    std::lock_guard<std::mutex> lock(request_mutex_);

    if (request_->agent_name == "omniverse")
    {
        // Process omniverse requests only once. (because omnivere cant send a request only once on startup, it keeps on sending the request while it is required to be sent only once)
        if (request_processed_)
        {
            RCLCPP_WARN(this->get_logger(), "STOD was sent previously to omniverse, you need to rerun the server for another omniverse request.");
            response_->success = false;
            return;
        }
        publishSTOD();
        request_processed_ = true; // Mark as processed
        response_->success = true;
    }

    else if (request_->agent_name == "hololens")
    {
        publishSTOD();
        response_->success = true;
    }

    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown agent %s", request_->agent_name.c_str());
        response_->success = false;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STODServer>());
    rclcpp::shutdown();
    return 0;
}