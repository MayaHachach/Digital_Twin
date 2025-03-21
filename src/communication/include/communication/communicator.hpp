#ifndef __COMMUNICATOR__HPP__
#define __COMMUNICATOR__HPP__

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <vector>
#include "communication/data_logger.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "customed_interfaces/msg/object.hpp" // Replace with the correct message type header
#include "customed_interfaces/msg/temp.hpp"
#include "customed_interfaces/srv/request_stod.hpp"
#include <cmath>

using namespace std;
class CommunicatorNode : public rclcpp::Node
{
private:
    //? subscribers
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr hololens_object_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr human_correction_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Temp>::SharedPtr temp_response_subscriber;
    vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subscribers_;

    //? publishers
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr omniverse_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr object_locations_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Temp>::SharedPtr temp_count_publisher;

    //? services
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_hololens_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_omniverse_publisher;
    rclcpp::Service<customed_interfaces::srv::RequestSTOD>::SharedPtr STOD_service;
    std::mutex request_mutex_;
    bool request_processed_;
    int request_processed_counter = 0;

    void handleRequest(const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
                       std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_);

    //? variables
    rclcpp::TimerBase::SharedPtr timer_;
    customed_interfaces::msg::Object::SharedPtr last_message_;
    customed_interfaces::msg::Temp::SharedPtr previous_message;
    customed_interfaces::msg::Object::SharedPtr message;
    DataLogger logger_;
    DataLogger temp_logger;
    double threshold_;
    double euclidean_threshold;
    double angular_threshold;
    float temp_id;

    std::unordered_map<std::string, int> object_counts{
        {"Husky", 2},
        {"MonitorEcho", 2},
        {"Kobuki", 1},
        {"Chair", 2},
        {"RedChair", 2}};

    struct topics_struct
    {
        string class_name;
        int id{0};
        nav_msgs::msg::Odometry odom_msg;
    };

    std::unordered_map<std::string, topics_struct> odom_topics =
        {
            {"/transformed_odom", {"Husky"}},
            {"/husky2/odom", {"Husky"}},
            {"kobuki/odom", {"Kobuki"}}};

    std::unordered_map<std::string, vector<DataLogger::object_map_struct>> object_map_;

    unordered_map<string, vector<customed_interfaces::msg::Object>> temp_map;
    std::vector<std::string> offline_objects = {"Chair", "MonitorEcho", "RedChair"};

    //? Methods
    void objectCallback(const customed_interfaces::msg::Object::SharedPtr msg);
    // void logAllObjects();
    // std::string GetTimestamp();
    void InitializePublishers();
    // void InitializeSubscribers();
    bool isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2);
    bool isMessageEqual(const customed_interfaces::msg::Temp &msg1, const customed_interfaces::msg::Temp &msg2);
    bool isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2);
    DataLogger::object_map_struct AddNewObject(const customed_interfaces::msg::Object &message);
    // services methods
    void publishHololensSTOD();
    void publishOmniverseSTOD();

    Eigen::Matrix4d poseToTransformation(const geometry_msgs::msg::Pose &pose);

public:
    CommunicatorNode(/* args */);
};

#endif //__COMMUNICATOR__HPP__