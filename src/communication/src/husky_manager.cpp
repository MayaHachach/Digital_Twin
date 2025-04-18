#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <vector>
// #include "customed_interfaces/msg/husky_status.hpp"
#include "husky_msgs/msg/husky_status.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

class HuskyOdometryTransformer : public rclcpp::Node
{
public:
    HuskyOdometryTransformer() : Node("husky_odometry_transformer")
    {
        // rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        // Subscribe to transformation data (relative position & yaw)
        transformation_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/transformation", 10, std::bind(&HuskyOdometryTransformer::transformation_callback, this, std::placeholders::_1));

        // Subscribe to pose data (robot's pose in its frame)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", 10, std::bind(&HuskyOdometryTransformer::pose_callback, this, std::placeholders::_1));

        status_sub = this->create_subscription<husky_msgs::msg::HuskyStatus>(
            "/status", qos_profile , std::bind(&HuskyOdometryTransformer::status_callback, this, std::placeholders::_1));
        
        husky_battery_pub = this->create_publisher<std_msgs::msg::Float32>("/husky/battery_percentage", 10);
        husky_velocity_pub = this->create_publisher<std_msgs::msg::Float32>("/husky/velocity", 10);
        husky_temperature_pub = this->create_publisher<std_msgs::msg::Float32>("/husky/temperature", 10);

        // Publisher for transformed odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/transformed_odom", 10);

        RCLCPP_INFO(this->get_logger(), "Husky Odometry Transformer Node Initialized");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr transformation_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<husky_msgs::msg::HuskyStatus>::SharedPtr status_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr husky_battery_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr husky_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr husky_temperature_pub;

    std::vector<double> last_position_ = {0.0, 0.0, 0.0};
    rclcpp::Time last_time_;
    bool first_pose_received_ = false;

    double relative_x_ = 0.0, relative_y_ = 0.0, relative_z_ = 0.0;
    double relative_yaw_ = 0.0; // In radians
    float battery_voltage, min_voltage = 12.0, max_voltage = 26.4;
    std_msgs::msg::Float32 driver_temp; 
    std_msgs::msg::Float32 battery_percentage;
    
    void transformation_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        relative_x_ = twist_msg->linear.x;
        relative_y_ = twist_msg->linear.y;
        relative_z_ = twist_msg->linear.z;
        relative_yaw_ = twist_msg->angular.z * M_PI / 180.0; // Convert degrees to radians

        RCLCPP_INFO(this->get_logger(), "Received Relative Position: [%.3f, %.3f, %.3f], Yaw: %.3f°",
                    relative_x_, relative_y_, relative_z_, twist_msg->angular.z);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!first_pose_received_)
        {
            last_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            last_time_ = msg->header.stamp;
            first_pose_received_ = true;
            return;
        }

        // Compute velocity
        std_msgs::msg::Float32 velocity = compute_velocity(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.stamp);
        husky_velocity_pub->publish(velocity);
        
        // Create and publish Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = msg->header;
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose = msg->pose;

        odom_pub_->publish(odom_msg);
        RCLCPP_INFO(this->get_logger(), "Published Transformed Odometry with Speed: %.3f m/s", velocity);
    }

    std_msgs::msg::Float32 compute_velocity(double x, double y, double z, rclcpp::Time current_time)
    {
        std_msgs::msg::Float32 velocity;
        double dx = x - last_position_[0];
        double dy = y - last_position_[1];
        double dz = z - last_position_[2];

        double dt = (current_time - last_time_).seconds();
        if (dt == 0)
            return velocity;

            double speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
            velocity.data = static_cast<float>(speed);
    
        // Update last position and time
        last_position_ = {x, y, z};
        last_time_ = current_time;

        return velocity;
    }

    void status_callback(const husky_msgs::msg::HuskyStatus husky_status_msg){
        battery_voltage = husky_status_msg.battery_voltage;
        battery_percentage.data = ((battery_voltage - min_voltage)/(max_voltage - min_voltage))*100;
        driver_temp.data = (husky_status_msg.left_driver_temp + husky_status_msg.right_driver_temp)/2;
        RCLCPP_INFO(this->get_logger(), "Battery voltage is: %f and the percentage is %d ", battery_voltage, battery_percentage);

        husky_battery_pub->publish(battery_percentage);
        husky_temperature_pub->publish(driver_temp);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HuskyOdometryTransformer>());
    rclcpp::shutdown();
    return 0;
}
