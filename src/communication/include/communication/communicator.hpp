#ifndef __COMMUNICATOR__HPP__
#define __COMMUNICATOR__HPP__

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <map>
#include <fstream>
#include <vector>
#include <string>
#include "communication/data_logger.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "customed_interfaces/msg/object.hpp" // Replace with the correct message type header
#include "customed_interfaces/msg/temp.hpp"
#include "customed_interfaces/srv/request_stod.hpp"
#include "customed_interfaces/srv/request_category.hpp"
#include "customed_interfaces/srv/edit_objects.hpp"

#include "communication/robot_monitor/robot_interface.hpp"
#include "communication/robot_monitor/husky_robot.hpp"
#include "communication/robot_monitor/kobuki_robot.hpp"
#include "communication/robot_monitor/robot_monitor_factory.hpp"
#include "customed_interfaces/msg/husky_status.hpp"
#include "customed_interfaces/msg/kobuki_status.hpp"
#include "yaml-cpp/yaml.h"
#include <filesystem> // C++17

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
// for transformation
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

// Structure to hold robot configuration
struct RobotConfig
{
    std::string type;
    std::map<std::string, std::string> feedback_topics;
    std::string odometry_topic;
};

class CommunicatorNode : public rclcpp::Node
{
private:
    //? subscribers
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr hololens_object_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr initial_STOD_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Object>::SharedPtr human_correction_subscriber;
    rclcpp::Subscription<customed_interfaces::msg::Temp>::SharedPtr temp_response_subscriber;
    vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subscribers_;
    vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> navigation_path_subscribers_;
    vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> navigation_status_subscribers_;

    //? publishers
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr omniverse_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr object_locations_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Temp>::SharedPtr temp_count_publisher;
    rclcpp::Publisher<customed_interfaces::msg::HuskyStatus>::SharedPtr object_status_publisher;
    rclcpp::Publisher<customed_interfaces::msg::KobukiStatus>::SharedPtr kobuki_status_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navigation_path_publisher;

    // Timer for publishing robot status
    rclcpp::TimerBase::SharedPtr status_publish_timer_;

    //? services
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_hololens_publisher;
    rclcpp::Publisher<customed_interfaces::msg::Object>::SharedPtr STOD_omniverse_publisher;
    rclcpp::Service<customed_interfaces::srv::RequestSTOD>::SharedPtr STOD_service;
    rclcpp::Service<customed_interfaces::srv::RequestCategory>::SharedPtr request_category_service_;
    rclcpp::Service<customed_interfaces::srv::EditObjects>::SharedPtr edit_object_service_;

    std::mutex request_mutex_;
    bool request_processed_;
    int request_processed_counter = 0;

    void handleRequest(const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
                       std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_);

    void handleRequestCategory(const std::shared_ptr<customed_interfaces::srv::RequestCategory::Request> request,
                               std::shared_ptr<customed_interfaces::srv::RequestCategory::Response> response);

    void handleEditObjectRequest(const std::shared_ptr<customed_interfaces::srv::EditObjects::Request> request,
                                 std::shared_ptr<customed_interfaces::srv::EditObjects::Response> response);

    bool publishSTODCategory(const std::string &category);

    //? variables
    rclcpp::TimerBase::SharedPtr timer_;
    customed_interfaces::msg::Object::SharedPtr last_message_;
    customed_interfaces::msg::Temp::SharedPtr previous_temp_message;
    customed_interfaces::msg::Object::SharedPtr message;
    int total_object_count = 0; // Total number of objects in the STOD

    DataLogger logger_;
    DataLogger temp_logger;
    double threshold_;
    double euclidean_threshold;
    double angular_threshold;
    float temp_id;
    bool stodReceived{false},
        prevStodReceived{false};
    std::unordered_map<std::string, int> object_counts;

    rclcpp::TimerBase::SharedPtr hololens_connection_timer_;
    bool hololens_connected{false};

    struct status_struct
    {
        string class_name;
        string robot_type;
    };

    //'b' for battery, 'v' for velocity, and 't' for temperature
    std::unordered_map<string, status_struct> status_topics;

    struct topics_struct
    {
        string class_name;
        int id;
        nav_msgs::msg::Odometry odom_msg;
        Eigen::Matrix4d T_human_correction = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_global_frame = Eigen::Matrix4d::Identity();
        bool recalculate_transformation = false;
    };

    struct navigation_struct
    {
        string class_name;
        string navigation_path_topic_name;
        string navigation_status_topic_name;

        int id;
        nav_msgs::msg::Path path_msg;
        bool recalculate_transformation = false;
    };

    // std::unordered_map<std::string, topics_struct> odom_topics =
    //     {
    //         {"/transformed_odom", {"Husky", 1}},
    //         {"/slam_odom", {"Kobuki", 1}}};

    std::unordered_map<std::string, topics_struct> odom_topics;

    std::unordered_map<std::string, navigation_struct> navigation_topics;

    std::unordered_map<std::string, vector<DataLogger::object_map_struct>> object_map_;

    unordered_map<string, vector<customed_interfaces::msg::Object>> temp_map;

    // Robot monitors
    std::map<std::string, std::shared_ptr<communication::RobotMonitor>> robot_monitors_; // [robot_name & id, robot_monitor_class]
    std::set<std::string> handled_goals_;

    //? Methods
    // general usage methods
    void InitializePublishers();
    void InitializeSubscribers();
    void InitializeServices();
    void PrintSTOD();
    void printMatrix(const Eigen::Matrix4d &matrix, const std::string &label);
    void fillObjectCounts();
    void fillOdomTopics();
    bool isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2);
    bool isMessageEqual(const customed_interfaces::msg::Temp &msg1, const customed_interfaces::msg::Temp &msg2);
    bool isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2);
    bool isObjectInSTOD(const std::string &object_class_name, int object_id = 0);
    void isHololensConnected();
    void processRobotsNode(const YAML::Node &robots, std::map<std::string, RobotConfig> &configs);
    void publishRobotStatus();
    DataLogger::object_map_struct AddNewObject(const customed_interfaces::msg::Object &message);
    DataLogger::object_map_struct AddObjectWithCount(const customed_interfaces::msg::Object &message);
    Eigen::Matrix4d poseToTransformation(const geometry_msgs::msg::Pose &pose);
    nav_msgs::msg::Odometry TransformationToPose(const Eigen::Matrix4d transformation_matrix);
    std::map<std::string, RobotConfig> loadRobotConfigs(const std::string &filename);
    bool isOdomReset(const std::string &topic_name, const builtin_interfaces::msg::Time &current_stamp);
    std::string uuid_to_string(const std::array<uint8_t, 16> &uuid);

    // callbacks
    void objectCallback(const customed_interfaces::msg::Object::SharedPtr msg);
    void objectUpdate(const customed_interfaces::msg::Object::SharedPtr msg);
    void humanCorrectionCallback(const customed_interfaces::msg::Object::SharedPtr human_corrected_msg);
    void tempResponseCallback(const customed_interfaces::msg::Temp::SharedPtr temp_response_msg);
    void STODExtractionCallback(const customed_interfaces::msg::Object::SharedPtr object_msg);
    void robotOdomCallback(const std::string &topic_name, const nav_msgs::msg::Odometry::SharedPtr msg);
    void navigationPathCallback(CommunicatorNode::navigation_struct &topicInfo, const nav_msgs::msg::Path::SharedPtr msg);
    void navigationStatusCallback(CommunicatorNode::navigation_struct &nav_data, const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    // services methods
    void publishHololensSTOD();
    void publishOmniverseSTOD();

public:
    void InitializeRobotMonitor();

    CommunicatorNode(/* args */);
    ~CommunicatorNode();
};

#endif //__COMMUNICATOR__HPP__