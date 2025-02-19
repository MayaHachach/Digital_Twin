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
#include <cmath>

using namespace std;
class CommunicatorNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr hololens_object_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Temp>::SharedPtr temp_response_subscriber;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr publisher_;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr object_locations_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Temp>::SharedPtr temp_count_publisher;
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

    vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subscribers_;

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
            {"/transformed_pose", {"Husky"}},
            {"/husky2/odom", {"Husky"}},
            {"kobuki/odom", {"Kobuki"}}};

    // let the map contain a struct that contains the message and id which is number of objects that have the same name like 2 huskies so id 1 and id 2
    // struct object_map_struct
    // {
    //     int id;
    //     string topic_name;
    //     customed_interfaces::msg::Object message;
    // };

    std::unordered_map<std::string, vector<DataLogger::object_map_struct>> object_map_;

    unordered_map<string, vector<customed_interfaces::msg::Object>> temp_map;
    std::vector<std::string> offline_objects = {"Chair", "MonitorEcho", "RedChair"};

    void objectCallback(const customed_interfaces::msg::Object::SharedPtr msg);
    // void logAllObjects();
    std::string GetTimestamp();
    bool isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2);
    bool isMessageEqual(const customed_interfaces::msg::Temp &msg1, const customed_interfaces::msg::Temp &msg2);
    bool isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2);
    DataLogger::object_map_struct AddNewObject(const customed_interfaces::msg::Object &message);
    // customed_interfaces::msg::Object getMsgWithID(const customed_interfaces::msg::Object &message);

public:
    CommunicatorNode(/* args */);
};

#endif //__COMMUNICATOR__HPP__