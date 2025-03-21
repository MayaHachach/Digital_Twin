#include "communication/communicator.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node"), logger_("initial_history"), temp_logger("temp_history")
{
    threshold_ = 0.1; // Allow small floating-point differences
    object_map_ = logger_.loadLastIterationToMap();
    temp_map = temp_logger.loadLastTempToMap();

    InitializePublishers();
    // InitializeServices();
    STOD_service = this->create_service<customed_interfaces::srv::RequestSTOD>(
        "request_STOD",
        std::bind(&CommunicatorNode::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

    STOD_hololens_publisher = this->create_publisher<customed_interfaces::msg::Object>("/hololensSTOD", 10);
    STOD_omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseSTOD", 10);

    hololens_object_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/hololensObject", 10,
    
        std::bind(&CommunicatorNode::objectCallback, this, std::placeholders::_1));

    human_correction_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/humanCorrection", 10, [this](const customed_interfaces::msg::Object::SharedPtr human_corrected_msg)
        {
        auto object_class = object_map_.find(human_corrected_msg->name);
        if (object_class == object_map_.end()){
            RCLCPP_INFO(this->get_logger(), "Class %s was not found in the object map", human_corrected_msg->name.c_str());
            return;
        }
        for (auto &object : object_class->second){
            if (object.message.id != human_corrected_msg->id){
                continue;
            }
            
            object.message.pose = human_corrected_msg->pose;

            RCLCPP_INFO(this->get_logger(), "%s %d pose was updated to [%.2f, %.2f, %.2f]",
                human_corrected_msg->name.c_str(), human_corrected_msg->id, human_corrected_msg->pose.position.x, human_corrected_msg->pose.position.y, human_corrected_msg->pose.position.z);
            
            omniverse_publisher->publish(object.message);
            logger_.logAllObjects(object_map_);
            break;

        } });

    temp_response_subscriber = this->create_subscription<customed_interfaces::msg::Temp>(
        "/tempResponse", 10, [this](const customed_interfaces::msg::Temp::SharedPtr temp_response_msg)
        {
            if (previous_message != nullptr && isMessageEqual(*temp_response_msg, *previous_message))
            {
                RCLCPP_INFO(this->get_logger(), "duplicate temp message");
                return;
            }
            previous_message = temp_response_msg;
            

            RCLCPP_INFO(this->get_logger(), "%s %d pose should be updated to [%.2f, %.2f, %.2f]",
                temp_response_msg->name.c_str(), temp_response_msg->number, temp_map.at(temp_response_msg->name).back().pose.position.x,temp_map.at(temp_response_msg->name).back().pose.position.y,temp_map.at(temp_response_msg->name).back().pose.position.z);
            
            auto it = object_map_.find(temp_response_msg->name);

            if (it == object_map_.end())
            {
                RCLCPP_INFO(this->get_logger(), "temp response object wasn't found in object_map");
                return;
            }

            for (auto &i : it->second)
            {
                // RCLCPP_INFO(this->get_logger(), "found");
                
                if (i.message.id != temp_response_msg->number)
                {
                    continue;
                }

                auto temp_it = temp_map.find(temp_response_msg->name);
                if (temp_it != temp_map.end() && !temp_it->second.empty()){
                    // Copy pose from the last object
                    i.message.pose = temp_it->second.back().pose;

                    omniverse_publisher->publish(i.message);

                    // Remove only the last object
                    temp_it->second.pop_back();
                    RCLCPP_INFO(this->get_logger(), "Last object removed from temp_map[%s]", temp_response_msg->name.c_str());
                    
                    logger_.logAllObjects(object_map_);
                    temp_logger.logTempObjects(temp_map);

                    break;
                }
            }
                        
            // publish temp object counts
            customed_interfaces::msg::Temp temp_message;
            temp_message.name = temp_response_msg->name;
            temp_message.number = temp_map[temp_response_msg->name].size();
            temp_count_publisher->publish(temp_message);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
            // publish object locations to hololens
            for (auto &object : object_map_[temp_response_msg->name])
            {
                
                object_locations_publisher->publish(object.message);
            }

            RCLCPP_INFO(this->get_logger(), "Current Temp Map:");
            for (const auto &pair : temp_map)
            {
                RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
                for (const auto &obj : pair.second)
                {
                    RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f]",
                                obj.id, obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
                }
            } });

    for (auto &pair : odom_topics)
    { // fill the odom_msg in odom_topics map
        robot_odom_subscribers_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            pair.first, 10,
            [this, &pair](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                pair.second.odom_msg = *msg;
                // if the object_map is empty
                auto it = object_map_.find(pair.second.class_name);
                if (it == object_map_.end())
                {
                    RCLCPP_INFO(this->get_logger(), "Class name %s not found in object_map_. Skipping.", pair.second.class_name.c_str());
                    return;
                }
                // if the topic_name is not yet linked to any object
                if (pair.second.id == 0)
                {
                    // search if that object is already available
                    //  auto it = object_map_.find(pair.second.class_name);
                    for (auto &i : it->second)
                    {
                        if (isPoseEqual(i.message.pose, msg->pose.pose))
                        {
                            RCLCPP_INFO(this->get_logger(), "topic will be linked with hololens object %s, and topic name %s", i.message.name.c_str(), pair.first.c_str());
                            pair.second.id = i.message.id;
                            i.topic_name = pair.first;
                        }
                    }
                    return;
                }
                // fill the object_map with its corresponding odom_msg from its odom_topic
                // auto it = object_map_.find(pair.second.class_name);
                for (auto &i : it->second)
                {
                    if (pair.second.id != i.message.id)
                    {
                        continue;
                    }

                    if (isPoseEqual(i.message.pose, msg->pose.pose))
                    {
                        RCLCPP_INFO(this->get_logger(), "pose is equal for %s", i.message.name.c_str());
                        return;
                    }
                    i.message.pose = msg->pose.pose;
                    omniverse_publisher->publish(i.message);
                    logger_.logAllObjects(object_map_);
                }
            }));
    }
}

void CommunicatorNode::objectCallback(const customed_interfaces::msg::Object::SharedPtr msg)
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
    auto it = object_map_.find(message->name);

    // if the object class is not in the map, create one
    if (it == object_map_.end())
    {
        RCLCPP_INFO(this->get_logger(), "creating new class..");
        DataLogger::object_map_struct new_object = AddNewObject(*message);
        logger_.logAllObjects(object_map_);
        return;
    }

    // check if the object is already in the map and in the same position
    for (auto &object : object_map_[message->name]) // go through the vector of structs
    {
        RCLCPP_INFO(this->get_logger(), "checking objects in class");

        if (!isPoseEqual(object.message.pose, message->pose))
        {
            continue;
        }

        // Check if topic_name is empty, if yes, fill it if available
        if (object.topic_name == "")
        {
            for (auto &pair : odom_topics)
            {
                // if the topic is already taken, skip
                if (pair.second.class_name != "")
                {
                    continue;
                }
                // if the pose is not approx equal, skip
                if (!isPoseEqual(pair.second.odom_msg.pose.pose, object.message.pose))
                {
                    continue;
                }
                pair.second.class_name = object.message.name;
                pair.second.id = object.message.id;

                object.topic_name = pair.first;

                RCLCPP_INFO(this->get_logger(), "edited the object topic_name to: %s", object.topic_name.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s is already available on same position. Skipping update.", msg->name.c_str());
        return;
    }

    ////////////////////////////
    if (object_counts[message->name] == 1)
    {
        RCLCPP_INFO(this->get_logger(), "There is only one %s, updating its pose...", message->name.c_str());
        object_map_.at(message->name)[0].message.pose = message->pose;
        omniverse_publisher->publish(object_map_[message->name][0].message);
        logger_.logAllObjects(object_map_);
        return;
    }

    // TODO: what to do if object_count > 1?
    // if the new id is greater than the object_count, save in temp

    if (object_map_[message->name].size() + 1 > object_counts[message->name] && find(offline_objects.begin(), offline_objects.end(), message->name) != offline_objects.end())
    {
        RCLCPP_INFO(this->get_logger(), "an extra %s was detected, checking with hololens", message->name.c_str());

        // add temp object to temp_map
        customed_interfaces::msg::Object new_temp_object = *message;
        for (auto &temp : temp_map[message->name])
        {
            if (isPoseEqual(temp.pose, message->pose))
            {
                RCLCPP_INFO(this->get_logger(), "temp object already exists in temp_map");
                return;
            }
        }
        new_temp_object.id = object_map_[message->name].size() + temp_map[message->name].size() + 1;
        temp_map[message->name].push_back(new_temp_object);
        RCLCPP_INFO(this->get_logger(), "new temp object of name = %s and id = %d is added to the temp_map", new_temp_object.name.c_str(), new_temp_object.id);
        RCLCPP_INFO(this->get_logger(), "Current Temp Map:");
        for (const auto &pair : temp_map)
        {
            RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
            for (const auto &obj : pair.second)
            {
                RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f]",
                            obj.id, obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
            }
        }

        // publish temp object counts
        customed_interfaces::msg::Temp temp_message;
        temp_message.name = message->name;
        temp_message.number = temp_map[message->name].size();
        temp_count_publisher->publish(temp_message);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // publish object locations to hololens
        for (auto &object : object_map_[message->name])
        {
            object_locations_publisher->publish(object.message);
        }

        // publish to omniverse
        // omniverse_publisher->publish(new_temp_object);
        temp_logger.logTempObjects(temp_map);

        return;
    }

    ////////////////////////////

    // if object is not found, create
    DataLogger::object_map_struct new_object = AddNewObject(*message);
    logger_.logAllObjects(object_map_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CommunicatorNode::InitializePublishers()
{
    omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseObject", 10);
    object_locations_publisher = this->create_publisher<customed_interfaces::msg::Object>("/objectLocations", 10);
    temp_count_publisher = this->create_publisher<customed_interfaces::msg::Temp>("/tempCount", 10);
}

DataLogger::object_map_struct CommunicatorNode::AddNewObject(const customed_interfaces::msg::Object &message)
{
    DataLogger::object_map_struct new_object;
    new_object.message = message;
    new_object.message.id = object_map_[message.name].size() + 1;

    RCLCPP_INFO(this->get_logger(), "message name: %s with size %ld", message.name.c_str(), object_map_[message.name].size());

    for (auto &pair : odom_topics)
    {
        if (pair.second.class_name != "")
        {
            continue;
        }

        if (!isPoseEqual(pair.second.odom_msg.pose.pose, message.pose))
        {
            continue;
        }
        pair.second.class_name = message.name;
        pair.second.id = new_object.message.id;

        new_object.topic_name = pair.first;

        RCLCPP_INFO(this->get_logger(), "added a new object with topic_name: %s", new_object.topic_name.c_str());
    }

    object_map_[message.name].push_back(new_object);
    RCLCPP_INFO(this->get_logger(), "Added new object: %s with id %d", message.name.c_str(), new_object.message.id);

    // Print the entire object_map_
    RCLCPP_INFO(this->get_logger(), "Current Object Map:");
    for (const auto &pair : object_map_)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
        }
    }

    // auto message_with_id = std::make_shared<customed_interfaces::msg::Object>(message);
    // message_with_id->name = message.name + std::to_string(new_object.message.id);
    omniverse_publisher->publish(new_object.message);
    return new_object;
}

bool CommunicatorNode::isMessageEqual(const customed_interfaces::msg::Object &msg1, const customed_interfaces::msg::Object &msg2)
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

bool CommunicatorNode::isMessageEqual(const customed_interfaces::msg::Temp &msg1, const customed_interfaces::msg::Temp &msg2)
{
    return msg1.name == msg2.name && msg1.number == msg2.number;
}

bool CommunicatorNode::isPoseEqual(const geometry_msgs::msg::Pose &msg1, const geometry_msgs::msg::Pose &msg2)
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommunicatorNode>());
    rclcpp::shutdown();
    return 0;
}
