#ifndef __DAY0_CREATION__HPP__
#define __DAY0_CREATION__HPP__

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include "customed_interfaces/msg/object.hpp" // Replace with the correct message type header
#include <cmath>

using namespace std;
class Day0Creation : public rclcpp::Node
{
private:
    //? subscribers
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr hololens_object_subscriber;

    //? publishers
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr omniverse_publisher;

    //? variables
    customed_interfaces::msg::Object::SharedPtr last_message_;
    customed_interfaces::msg::Object::SharedPtr message;
    double threshold_;
    double euclidean_threshold;
    double angular_threshold;

    struct day0_object_map_struct
    {
        string topic_name;
        customed_interfaces::msg::Object message;
    };

    unordered_map<string, vector<day0_object_map_struct>> day0_object_map;

    std::vector<std::string> offline_objects = {"Chair", "MonitorEcho", "RedChair"};

    //? Methods
    // general usage methods
    void InitializePublishers();
    void InitializeSubscribers();
    void PrintSTOD();
    bool isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2);
    bool isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2);
    bool isObjectInSTOD(const std::string &object_class_name, int object_id = 0);
    day0_object_map_struct AddNewObject(const customed_interfaces::msg::Object &message);

    // callbacks
    void objectCallback(const customed_interfaces::msg::Object::SharedPtr msg);

public:
    Day0Creation(/* args */);
};

#endif //__DAY0_CREATION__HPP__