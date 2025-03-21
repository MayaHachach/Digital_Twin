#ifndef __REQUEST__HISTORY_SERVER__
#define __REQUEST__HISTORY__SERVER__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>
#include "customed_interfaces/srv/request_history.hpp"
#include "communication/communicator.hpp"

using namespace std;
class HistoryFileServer : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Service<customed_interfaces::srv::RequestHistory>::SharedPtr service_;

    void handleRequest(
        const std::shared_ptr<customed_interfaces::srv::RequestHistory::Request> request,
        std::shared_ptr<customed_interfaces::srv::RequestHistory::Response> response);

public:
    HistoryFileServer(/* args */);
};

#endif