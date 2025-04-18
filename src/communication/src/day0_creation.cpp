#include "communication/day0_creation.hpp"

Day0Creation::Day0Creation() : Node("day0_creation")
{
    threshold_ = 0.1; // Allow small floating-point differences

    InitializePublishers();
    InitializeSubscribers();
}

void Day0Creation::InitializePublishers()
{
    omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseObject", 10);
}

void Day0Creation::InitializeSubscribers()
{
    hololens_object_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/hololensObject", 10,
        std::bind(&Day0Creation::objectCallback, this, std::placeholders::_1));
}

void Day0Creation::objectCallback(const customed_interfaces::msg::Object::SharedPtr msg)
{
    // Check if the message is the same as the last one
    if (last_message_ != nullptr && isMessageEqual(*msg, *last_message_))
    {
        RCLCPP_INFO(this->get_logger(), "duplicate message");
        return;
    }

    message = last_message_ = msg; // Store the received message
    RCLCPP_INFO(this->get_logger(), "new message is received");

    // check if object class if found in our object_map
    auto it = day0_object_map.find(message->name);

    // if the object class is not in the map, create one
    if (it == day0_object_map.end())
    {
        RCLCPP_INFO(this->get_logger(), "creating new class..");
        day0_object_map_struct new_object = AddNewObject(*message);
        return;
    }

    // check if the object is already in the map and in the same position
    for (auto &object : day0_object_map[message->name]) // go through the vector of structs
    {
        RCLCPP_INFO(this->get_logger(), "checking objects in class");

        if (!isPoseEqual(object.message.pose, message->pose))
        {
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "%s is already available on same position. Skipping update.", msg->name.c_str());
        return;
    }

    // if object is not found, create
    day0_object_map_struct new_object = AddNewObject(*message);
}

bool Day0Creation::isObjectInSTOD(const std::string &object_class_name, int object_id)
{
    auto it = day0_object_map.find(object_class_name);
    if (it == day0_object_map.end())
        return false;

    // If object_id is 0, we only check if the class exists
    if (object_id == 0)
        return true;

    // Otherwise, check if the specific ID exists in that class
    const auto &objects = it->second;
    for (const auto &obj : objects)
    {
        if (obj.message.id == object_id)
            return true;
    }

    return false;
}

Day0Creation::day0_object_map_struct Day0Creation::AddNewObject(const customed_interfaces::msg::Object &message)
{
    day0_object_map_struct new_object;
    new_object.message = message;
    new_object.message.id = day0_object_map[message.name].size() + 1;

    day0_object_map[message.name].push_back(new_object);
    RCLCPP_INFO(this->get_logger(), "Added new object: %s with id %d", message.name.c_str(), new_object.message.id);
    // Print the entire object_map_
    PrintSTOD();
    omniverse_publisher->publish(new_object.message);
    return new_object;
}

bool Day0Creation::isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2)
{
    euclidean_threshold = 0.2;
    angular_threshold = 0.5; // radians => 30 degrees
    // Extract positions
    double x1 = msg1.position.x;
    double y1 = msg1.position.y;
    double z1 = msg1.position.z;

    double x2 = msg2.position.x;
    double y2 = msg2.position.y;
    double z2 = msg2.position.z;

    // Compute Euclidean distance
    double distance = std::sqrt(std::pow(x1 - x2, 2) +
                                std::pow(y1 - y2, 2) +
                                std::pow(z1 - z2, 2));

    // if pose is approx equal, check the orientation
    if (distance < euclidean_threshold)
    {
        // Extract quaternions
        double x12 = msg1.orientation.x;
        double y12 = msg1.orientation.y;
        double z12 = msg1.orientation.z;
        double w12 = msg1.orientation.w;

        double x22 = msg2.orientation.x;
        double y22 = msg2.orientation.y;
        double z22 = msg2.orientation.z;
        double w22 = msg2.orientation.w;

        // Compute quaternion dot product
        double dot_product = x12 * x22 + y12 * y22 + z12 * z22 + w12 * w22;

        // Clamp value to avoid NaN issues due to floating-point precision
        dot_product = std::max(-1.0, std::min(1.0, dot_product));

        // Compute the angular distance (radians)
        double angle_difference = std::acos(dot_product) * 2.0; // Convert to full rotation difference

        // Compare against a threshold (e.g., 0.1 radians ~5.7 degrees)
        return angle_difference < angular_threshold;
    }
    else
    {
        return distance < euclidean_threshold;
    }
}

bool Day0Creation::isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2)
{
    // Compare all fields of the message to determine equality
    return msg1.name == msg2.name &&
           std::abs(msg1.pose.position.x - msg2.pose.position.x) < threshold_ &&
           std::abs(msg1.pose.position.y - msg2.pose.position.y) < threshold_ &&
           std::abs(msg1.pose.position.z - msg2.pose.position.z) < threshold_ &&
           std::abs(msg1.pose.orientation.x - msg2.pose.orientation.x) < threshold_ &&
           std::abs(msg1.pose.orientation.y - msg2.pose.orientation.y) < threshold_ &&
           std::abs(msg1.pose.orientation.z - msg2.pose.orientation.z) < threshold_ &&
           std::abs(msg1.pose.orientation.w - msg2.pose.orientation.w) < threshold_ &&
           std::abs(msg1.scale.x - msg2.scale.x) < threshold_ &&
           std::abs(msg1.scale.y - msg2.scale.y) < threshold_ &&
           std::abs(msg1.scale.z - msg2.scale.z) < threshold_;
}

void Day0Creation::PrintSTOD()
{
    RCLCPP_INFO(this->get_logger(), "Current STOD:");
    for (const auto &pair : day0_object_map)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Day0Creation>());
    rclcpp::shutdown();
    return 0;
}