#include "communication/communicator.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node"), logger_("initial_history"), temp_logger("temp_history")
{
    threshold_ = 0.1; // Allow small floating-point differences
    object_map_ = logger_.loadLastIterationToMap();
    temp_map = temp_logger.loadLastTempToMap();

    PrintSTOD();
    fillObjectCounts();

    // initial_STOD_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
    //     "/initialSTOD", 10,
    //     std::bind(&CommunicatorNode::STODExtractionCallback, this, std::placeholders::_1));
    // InitializeServices();

    // Create a timer to publish robot status data
    status_publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), // 1 second interval
        std::bind(&CommunicatorNode::publishRobotStatus, this));

    RCLCPP_INFO(this->get_logger(), "Communicator node initialized with robot monitors");
}

// Monitoring

// Helper function to process the robots node
void CommunicatorNode::processRobotsNode(const YAML::Node &robots, std::map<std::string, RobotConfig> &configs)
{
    for (const auto &robot : robots)
    {
        RobotConfig robot_config;
        std::string robot_id = robot.first.as<std::string>();

        const auto &robot_data = robot.second;
        robot_config.type = robot_data["type"].as<std::string>();
        RCLCPP_INFO(this->get_logger(), "Robot Type: %s", robot_config.type.c_str());
        // Handle odometry topic
        if (robot_data["topics"] && robot_data["topics"]["odometry"])
        {
            robot_config.odometry_topic = robot_data["topics"]["odometry"].as<std::string>();
        }

        // Handle feedback topics
        if (robot_data["topics"] && robot_data["topics"]["feedback"])
        {
            const auto &feedback = robot_data["topics"]["feedback"];
            for (const auto &topic : feedback)
            {
                auto topic_type = topic.first.as<std::string>();
                auto topic_name = topic.second.as<std::string>();
                robot_config.feedback_topics[topic_type] = topic_name;
                RCLCPP_INFO(this->get_logger(), "Feedback Topic: %s -> %s",
                            topic_type.c_str(), topic_name.c_str());
            }
        }

        configs[robot_id] = robot_config;
        RCLCPP_INFO(this->get_logger(),
                    "Loaded configuration for robot %s: type=%s, odom=%s, feedback_topics=%zu",
                    robot_id.c_str(),
                    robot_config.type.c_str(),
                    robot_config.odometry_topic.c_str(),
                    robot_config.feedback_topics.size());

        RCLCPP_INFO(this->get_logger(), "--------------------------------------");
    }
}

std::map<std::string, RobotConfig> CommunicatorNode::loadRobotConfigs(const std::string &filename)
{
    std::map<std::string, RobotConfig> configs;

    try
    {
        YAML::Node config = YAML::LoadFile(filename);
        RCLCPP_INFO(this->get_logger(), "Loading robots from communicator_node");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------");
        const auto &robots = config["communicator_node"]["ros_parameters"]["robots"];

        processRobotsNode(robots, configs);
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error("Error parsing YAML file: " + std::string(e.what()));
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Error loading robot configurations: " + std::string(e.what()));
    }

    return configs;
}

void CommunicatorNode::InitializeRobotMonitor()
{
    // Load robot configurations from YAML file
    std::string config_path = this->declare_parameter("config_path", "config/robots.yaml");
    std::map<std::string, RobotConfig> robot_configs = loadRobotConfigs(config_path);

    // Initialize robot monitors with the loaded configurations
    try
    {
        // Create monitors for each robot
        for (const auto &[robot_id, config] : robot_configs)
        {
            RCLCPP_INFO(this->get_logger(), "Creating monitor for robot %s of type %s",
                        robot_id.c_str(), config.type.c_str());
            auto monitor = communication::createRobotMonitor(robot_id, config.type, config.feedback_topics, this->shared_from_this());
            if (monitor)
            {
                robot_monitors_[robot_id] = monitor; // store monitor instance 
                RCLCPP_INFO(this->get_logger(), "Created monitor for robot %s of type %s",
                            robot_id.c_str(), config.type.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create monitor for robot %s",
                             robot_id.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "--------------------------------------");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error initializing robot monitors: %s", e.what());
    }

    //! Process robot configurations for topics
    // for (const auto &[topic_name, robot_config] : robot_configs)
    // {
    //     // Add odometry topic
    //     if (!robot_config.odometry_topic.empty())
    //     {
    //         odom_topics[robot_config.odometry_topic] = topics_struct();
    //         odom_topics[robot_config.odometry_topic].class_name = robot_config.type;
    //         odom_topics[robot_config.odometry_topic].id = 1;
    //         RCLCPP_INFO(this->get_logger(), "Added odometry topic: %s",
    //                     robot_config.odometry_topic.c_str());
    //     }
    // }
}

void CommunicatorNode::publishRobotStatus()
{
    RCLCPP_INFO(this->get_logger(), "publishRobotStatus");

    // Publish status for each robot
    for (const auto &[robot_id, monitor] : robot_monitors_)
    {
        const auto &status = monitor->getStatus();
        if (status.robot_type == "husky")
        {
            customed_interfaces::msg::HuskyStatus msg;
            msg.name = status.robot_id;
            msg.battery_percentage = status.data["battery_level"];

            // Add additional data if available
            auto temp_it = status.data.find("left_motor_temp");
            if (temp_it != status.data.end())
            {
                msg.left_motor_temp = temp_it->second;
            }

            auto current_it = status.data.find("left_driver_current");
            if (current_it != status.data.end())
            {
                msg.left_driver_current = current_it->second;
            }

            // object_status_publisher_->publish(msg);
            object_status_publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published Husky status for robot %s", robot_id.c_str());
        }
        else if (status.robot_type == "kobuki")
        {
            auto msg = std::make_unique<customed_interfaces::msg::KobukiStatus>();
            msg->name = status.robot_id;
            // msg->battery_percentage = status.data.find("battery_percentage");

            // Add additional data if available
            auto left_current_it = status.data.find("left_driver_current");
            if (left_current_it != status.data.end())
            {
                msg->left_driver_current = left_current_it->second;
            }

            auto right_current_it = status.data.find("right_driver_current");
            if (right_current_it != status.data.end())
            {
                msg->right_driver_current = right_current_it->second;
            }

            kobuki_status_publisher->publish(std::move(msg));
            RCLCPP_INFO(this->get_logger(), "Published Kobuki status for robot %s", robot_id.c_str());
        }
    }
}

// Subscription Callbacks

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
        DataLogger::object_map_struct new_object = AddObjectWithCount(*message);
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

    // if (object_map_[message->name].size() + 1 > object_counts[message->name] && find(offline_objects.begin(), offline_objects.end(), message->name) != offline_objects.end())
    if (object_map_[message->name].size() + 1 > object_counts[message->name] && object_map_.at(message->name)[message->id - 1].topic_name.empty())
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
    DataLogger::object_map_struct new_object = AddObjectWithCount(*message);
    logger_.logAllObjects(object_map_);
}

void CommunicatorNode::objectUpdate(const customed_interfaces::msg::Object::SharedPtr msg)
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
    if (!isObjectInSTOD(message->name, message->id))
    {
        RCLCPP_INFO(this->get_logger(), "object doesn't exist in STOD");
        return;
    }

    // check if there is only one instance of the object
    if (object_counts[message->name] == 1)
    {
        auto object_instance = object_map_.at(message->name)[0];
        if (isPoseEqual(object_instance.message.pose, message->pose))
        {
            RCLCPP_INFO(this->get_logger(), "%s %d is still in the same pose", message->name, message->id);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "There is only one %s, updating its pose...", message->name.c_str());

        object_map_.at(message->name)[0].message.pose = message->pose;
        auto &topicInfo = odom_topics[object_instance.topic_name];
        topicInfo.recalculate_transformation = true;

        omniverse_publisher->publish(object_map_[message->name][0].message);
        logger_.logAllObjects(object_map_);
        PrintSTOD();
        return;
    }

    ///////////////////////////////////////////////////

    // if the object is an online object, id !=0 when received from hololens
    if (message->id != 0)
    {
        auto &online_object = object_map_.at(message->name)[message->id - 1];
        if (isPoseEqual(online_object.message.pose, message->pose))
        {
            RCLCPP_INFO(this->get_logger(), "%s %d is still in the same pose", message->name.c_str(), message->id);
            return;
        }
        // replace new location in STOD and recalculate the global transformation matrix
        online_object.message.pose = message->pose;
        auto &topicInfo = odom_topics[online_object.topic_name];
        topicInfo.recalculate_transformation = true;

        // publish and log
        omniverse_publisher->publish(online_object.message);
        logger_.logAllObjects(object_map_);
        PrintSTOD();
        return;
    }

    ///////////////////////////////////////////////

    // if the object is an offline object, id = 0
    auto it = object_map_.find(message->name);

    // check if the object is already in the map and in the same position
    for (auto &object : object_map_[message->name]) // go through the vector of structs
    {
        RCLCPP_INFO(this->get_logger(), "checking objects in class");

        if (!isPoseEqual(object.message.pose, message->pose))
        {
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "%s is already available on same position. Skipping update.", msg->name.c_str());
        return;
    }

    // if the offline object is new, create a temp object (offline object handling)

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

void CommunicatorNode::robotOdomCallback(const std::string &topic_name, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto &topicInfo = odom_topics[topic_name];

    if (!isObjectInSTOD(topicInfo.class_name, topicInfo.id))
    {
        RCLCPP_INFO(this->get_logger(), "%s %d was not found in STOD for odom update", topicInfo.class_name.c_str(), topicInfo.id);
        return;
    }

    auto &object = object_map_.at(topicInfo.class_name)[topicInfo.id - 1];
    object.topic_name = topic_name;

    Eigen::Matrix4d T_odom = poseToTransformation(msg->pose.pose);

    //! IMPORTANT
    // TODO: change the T_odom,isIdentity to detect if the publisher stopped publishing to know if the robot reinitialized
    if (topicInfo.T_global_frame.isIdentity() || T_odom.isIdentity() || topicInfo.recalculate_transformation)
    {
        Eigen::Matrix4d T_object = poseToTransformation(object.message.pose);
        topicInfo.T_global_frame = T_object * T_odom.inverse();
        RCLCPP_INFO(this->get_logger(), "Calculating matrix for %s %d", topicInfo.class_name.c_str(), topicInfo.id);
        printMatrix(topicInfo.T_global_frame, "T_global_frame");

        topicInfo.T_human_correction = Eigen::Matrix4d::Identity();
        topicInfo.recalculate_transformation = false;
    }

    Eigen::Matrix4d T_correct_odom = topicInfo.T_human_correction * topicInfo.T_global_frame * T_odom;

    topicInfo.odom_msg = TransformationToPose(T_correct_odom);

    if (isPoseEqual(object.message.pose, topicInfo.odom_msg.pose.pose))
        return;

    object.message.pose = topicInfo.odom_msg.pose.pose;

    omniverse_publisher->publish(object.message);
    PrintSTOD();
    logger_.logAllObjects(object_map_);
}

void CommunicatorNode::tempResponseCallback(const customed_interfaces::msg::Temp::SharedPtr temp_response_msg)
{
    if (previous_temp_message != nullptr && isMessageEqual(*temp_response_msg, *previous_temp_message))
    {
        RCLCPP_INFO(this->get_logger(), "duplicate temp message");
        return;
    }
    previous_temp_message = temp_response_msg;

    RCLCPP_INFO(this->get_logger(), "%s %d pose should be updated to [%.2f, %.2f, %.2f]",
                temp_response_msg->name.c_str(), temp_response_msg->number, temp_map.at(temp_response_msg->name).back().pose.position.x, temp_map.at(temp_response_msg->name).back().pose.position.y, temp_map.at(temp_response_msg->name).back().pose.position.z);

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
        if (temp_it != temp_map.end() && !temp_it->second.empty())
        {
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
    }
}

void CommunicatorNode::humanCorrectionCallback(const customed_interfaces::msg::Object::SharedPtr human_corrected_msg)
{
    if (!isObjectInSTOD(human_corrected_msg->name, human_corrected_msg->id))
        return;

    // class is found
    auto &object = object_map_.at(human_corrected_msg->name)[human_corrected_msg->id - 1];

    if (!object.topic_name.empty())
    { // online object that has a topic
        // find odom_topic
        auto odometry_topic = odom_topics.find(object.topic_name);

        // calculate transformation matrix
        // Eigen::Matrix4d T_odom = poseToTransformation(odometry_topic->second.odom_msg.pose.pose);
        Eigen::Matrix4d T_object = poseToTransformation(object.message.pose);
        Eigen::Matrix4d T_corrected = poseToTransformation(human_corrected_msg->pose);

        // Compute the correction transformation
        odometry_topic->second.T_human_correction = T_corrected * T_object.inverse();
        RCLCPP_INFO(this->get_logger(), "human correction was calculated by multiplying T_corrected * T_object.inverse()");
    }

    RCLCPP_INFO(this->get_logger(), "Human agent suggested a correction to %s %d to position [%.2f, %.2f, %.2f]",
                human_corrected_msg->name.c_str(), human_corrected_msg->id, human_corrected_msg->pose.position.x, human_corrected_msg->pose.position.y, human_corrected_msg->pose.position.z);

    // replace the current pose with the corrected pose
    object.message.pose = human_corrected_msg->pose; // applies for both online and offline objects
    PrintSTOD();
    omniverse_publisher->publish(object.message);
    logger_.logAllObjects(object_map_);
}

void CommunicatorNode::STODExtractionCallback(const customed_interfaces::msg::Object::SharedPtr object_msg)
{
    if (!object_counts.empty()) // already object_map exists, no need to overwrite it, turn off subscriber.
    {
        initial_STOD_subscriber.reset();
        return;
    }

    if (object_msg->name == "end")
    {
        // object count
        for (auto &pair : object_map_)
        {
            object_counts[pair.first] = pair.second.size();

            // sort the vector
            std::sort(pair.second.begin(), pair.second.end(), [this](const DataLogger::object_map_struct &a, const DataLogger::object_map_struct &b)
                      { return a.message.id < b.message.id; });

            PrintSTOD();
        }

        // print object_counts
        for (const auto &object_count : object_counts)
        {
            RCLCPP_INFO(this->get_logger(), "Object count of class %s is %d", object_count.first.c_str(), object_count.second);
        }

        // logging
        logger_.logAllObjects(object_map_);
        stodReceived = true;
        return;
    }

    if (isObjectInSTOD(object_msg->name, object_msg->id))
        return;

    AddNewObject(*object_msg);
}

// Service Handlers
void CommunicatorNode::handleRequest(const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
                                     std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_)
{
    std::lock_guard<std::mutex> lock(request_mutex_); // While the callback function is running, no other thread can access variables.

    if (request_->agent_name == "omniverse")
    {
        // Process omniverse requests only once. (because omnivere cant send a request only once on startup, it keeps on sending the request while it is required to be sent only once)
        if (request_processed_)
        {
            if (request_processed_counter == 0)
            {
                RCLCPP_WARN(this->get_logger(), "STOD was sent previously to omniverse, you need to rerun the server for another omniverse request.");
                request_processed_counter = 1;
            }
            response_->success = false;
            return;
        }

        publishOmniverseSTOD();
        request_processed_ = true; // Mark as processed
        response_->success = true;
    }

    else if (request_->agent_name == "hololens")
    {
        publishHololensSTOD();
        response_->success = true;
    }

    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown agent %s", request_->agent_name.c_str());
        response_->success = false;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Methods
void CommunicatorNode::InitializePublishers()
{
    omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseObject", 10);
    object_locations_publisher = this->create_publisher<customed_interfaces::msg::Object>("/objectLocations", 10);
    STOD_hololens_publisher = this->create_publisher<customed_interfaces::msg::Object>("/hololensSTOD", 10);
    STOD_omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseSTOD", 10);
    temp_count_publisher = this->create_publisher<customed_interfaces::msg::Temp>("/tempCount", 10);

    object_status_publisher = this->create_publisher<customed_interfaces::msg::HuskyStatus>("/objectStatus", 10);
    kobuki_status_publisher = this->create_publisher<customed_interfaces::msg::KobukiStatus>("/kobukiStatus", 10);
}

void CommunicatorNode::InitializeSubscribers()
{
    // !!!!!!
    // hololens_object_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
    //     "/hololensObject", 10,
    //     std::bind(&CommunicatorNode::objectCallback, this, std::placeholders::_1));

    hololens_object_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/hololensObject", 10,
        std::bind(&CommunicatorNode::objectUpdate, this, std::placeholders::_1));

    human_correction_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/humanCorrection", 10, std::bind(&CommunicatorNode::humanCorrectionCallback, this, std::placeholders::_1));

    temp_response_subscriber = this->create_subscription<customed_interfaces::msg::Temp>(
        "/tempResponse", 10, std::bind(&CommunicatorNode::tempResponseCallback, this, std::placeholders::_1));

    initial_STOD_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/initialSTOD", 10,
        std::bind(&CommunicatorNode::STODExtractionCallback, this, std::placeholders::_1));

    for (auto &pair : odom_topics)
    {
        const std::string &topic_name = pair.first;

        robot_odom_subscribers_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, 10,
            [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->robotOdomCallback(topic_name, msg);
            }));
    }
}

void CommunicatorNode::InitializeServices()
{
    STOD_service = this->create_service<customed_interfaces::srv::RequestSTOD>(
        "request_STOD",
        std::bind(&CommunicatorNode::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
}

void CommunicatorNode::fillObjectCounts()
{
    if (object_map_.empty())
        return;

    for (auto &pair : object_map_)
    {
        object_counts[pair.first] = pair.second.size();
        RCLCPP_INFO(this->get_logger(), "Object count of class %s is %d", pair.first.c_str(), object_counts[pair.first]);
    }
}

// void CommunicatorNode::StatusSubscribers()
// {
//     for (auto &status_topic_name : status_topics)
//     {
//         // TODO: add a condition if the robot isnt found, make a function for it
//         if (status_topic_name.second.class_name == "Husky")
//         {
//             RCLCPP_INFO(this->get_logger(), "status_topic_name is Husky");
//             if (status_topic_name.second.status_type == "battery")
//             {
//                 RCLCPP_INFO(this->get_logger(), "status_type is battery");
//
//                 this->create_subscription<std_msgs::msg::Float32>(
//                     status_topic_name.first, 10,
//                     [this, &status_topic_name](const std_msgs::msg::Float32::SharedPtr msg)
//                     {
//                         auto &object = object_map_.at(status_topic_name.second.class_name)[status_topic_name.second.id - 1];
//                         object.status.husky_status.name = object.message.name;
//                         object.status.husky_status.id = object.message.id;
//                         object.status.husky_status.battery_percentage = msg->data;
//                         RCLCPP_INFO(this->get_logger(), "husky status is %d", object.status.husky_status.battery_percentage);
//                         object_status_publisher->publish(object.status.husky_status);
//                     });
//             }
//             else if (status_topic_name.second.status_type == "status")
//             {
//                 this->create_subscription<customed_interfaces::msg::HuskyStatus>(
//                     status_topic_name.first, 10,
//                     [this, &status_topic_name](const customed_interfaces::msg::HuskyStatus msg)
//                     {
//                         auto &object = object_map_.at(status_topic_name.second.class_name)[status_topic_name.second.id - 1];
//                         object.status.type = "Husky";
//                         object.status.husky_status.name = object.message.name;
//                         object.status.husky_status.id = object.message.id;
//                         object.status.husky_status.left_driver_current = msg.left_driver_current;
//                         object.status.husky_status.right_driver_current = msg.right_driver_current;
//                         object.status.husky_status.battery_voltage = msg.battery_voltage;
//                         object.status.husky_status.left_driver_voltage = msg.left_driver_voltage;
//                         object.status.husky_status.right_driver_voltage = msg.right_driver_voltage;
//                         object.status.husky_status.left_driver_temp = msg.left_driver_temp;
//                         object.status.husky_status.right_driver_temp = msg.right_driver_temp;
//                         object.status.husky_status.timeout = msg.timeout;
//                         object.status.husky_status.lockout = msg.lockout;
//                         object.status.husky_status.e_stop = msg.e_stop;
//                         object.status.husky_status.ros_pause = msg.ros_pause;
//                         object.status.husky_status.no_battery = msg.no_battery;
//                         object.status.husky_status.current_limit = msg.current_limit;
//                         object_status_publisher->publish(object.status.husky_status);
//                     });
//             }
//         }
//         else if (status_topic_name.second.class_name == "Kobuki")
//         {
//         }
//     }
// for (auto &status_topic_name : status_topics)
// { // fill the odom_msg in odom_topics map
//     robot_status_subscribers_.push_back(this->create_subscription<std_msgs::msg::Float32>(
//         status_topic_name.first, 10,
//         [this, &status_topic_name](const std_msgs::msg::Float32::SharedPtr msg)
//         {
//             auto &object = object_map_.at(status_topic_name.second.class_name)[status_topic_name.second.id - 1];
//             // TODO: create a condition if object is not found
//             //  RCLCPP_INFO(this->get_logger(), object.message.name.c_str());
//             object.status.name = object.message.name;
//             object.status.id = object.message.id;
//             if (status_topic_name.second.status_type == "battery")
//             {
//                 object.status.battery_percentage = msg->data;
//             }
//             else if (status_topic_name.second.status_type == "velocity")
//             {
//                 object.status.velocity = msg->data;
//             }
//             else if (status_topic_name.second.status_type == "temperature")
//             {
//                 object.status.temperature = msg->data;
//             }
//             object_status_publisher->publish(object.status);
//             // TODO: log objects based on time
//         }));
// }
// }

bool CommunicatorNode::isObjectInSTOD(const std::string &object_class_name, int object_id)
{
    auto it = object_map_.find(object_class_name);
    if (it == object_map_.end())
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

DataLogger::object_map_struct CommunicatorNode::AddNewObject(const customed_interfaces::msg::Object &message)
{
    DataLogger::object_map_struct new_object;
    new_object.message = message;

    for (auto &[topic_name, topic_struct] : odom_topics)
    {
        if (topic_struct.class_name == new_object.message.name && topic_struct.id == new_object.message.id)
        {
            // Check if topic_name is empty, if yes, fill it if available
            if (new_object.topic_name.empty())
            {
                new_object.topic_name = topic_name;
                break;
            }
        }
    }

    object_map_[message.name].push_back(new_object);
    RCLCPP_INFO(this->get_logger(), "Added new object: %s with id %d", message.name.c_str(), new_object.message.id);
    // Print the entire object_map_
    PrintSTOD();
    omniverse_publisher->publish(new_object.message);
    RCLCPP_INFO(this->get_logger(), "DEBUGGING");
    return new_object;
}

DataLogger::object_map_struct CommunicatorNode::AddObjectWithCount(const customed_interfaces::msg::Object &message)
{
    DataLogger::object_map_struct new_object;
    new_object.message = message;
    new_object.message.id = object_map_[message.name].size() + 1;

    for (auto &[topic_name, topic_struct] : odom_topics)
    {
        if (topic_struct.class_name == new_object.message.name && topic_struct.id == new_object.message.id)
        {
            // Check if topic_name is empty, if yes, fill it if available
            if (new_object.topic_name.empty())
            {
                new_object.topic_name = topic_name;
                break;
            }
        }
    }

    object_map_[message.name].push_back(new_object);
    RCLCPP_INFO(this->get_logger(), "Added new object: %s with id %d", message.name.c_str(), new_object.message.id);
    // Print the entire object_map_
    PrintSTOD();
    omniverse_publisher->publish(new_object.message);
    return new_object;
}

void CommunicatorNode::PrintSTOD()
{
    RCLCPP_INFO(this->get_logger(), "Current STOD:");
    for (const auto &pair : object_map_)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
        }
    }
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

Eigen::Matrix4d CommunicatorNode::poseToTransformation(const geometry_msgs::msg::Pose &pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Set translation
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;

    // Convert quaternion to rotation matrix
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 rotation_matrix(q);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T(i, j) = rotation_matrix[i][j];
        }
    }

    return T;
}

nav_msgs::msg::Odometry CommunicatorNode::TransformationToPose(const Eigen::Matrix4d transformation_matrix)
{
    // Convert back to a Pose message
    nav_msgs::msg::Odometry corrected_pose;
    corrected_pose.pose.pose.position.x = transformation_matrix(0, 3);
    corrected_pose.pose.pose.position.y = transformation_matrix(1, 3);
    corrected_pose.pose.pose.position.z = transformation_matrix(2, 3);

    // Extract rotation as quaternion
    Eigen::Matrix3d R_corrected = transformation_matrix.block<3, 3>(0, 0);
    Eigen::Quaterniond q_corrected(R_corrected);
    corrected_pose.pose.pose.orientation.x = q_corrected.x();
    corrected_pose.pose.pose.orientation.y = q_corrected.y();
    corrected_pose.pose.pose.orientation.z = q_corrected.z();
    corrected_pose.pose.pose.orientation.w = q_corrected.w();

    return corrected_pose;
}

void CommunicatorNode::publishHololensSTOD()
{
    RCLCPP_INFO(this->get_logger(), "Processing request and publishing objects...");
    for (const auto &pair : object_map_)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
            STOD_hololens_publisher->publish(obj.message);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /STOD", object_map_.size());
}

void CommunicatorNode::publishOmniverseSTOD()
{
    RCLCPP_INFO(this->get_logger(), "Processing request and publishing objects...");
    for (const auto &pair : object_map_)
    {
        RCLCPP_INFO(this->get_logger(), "Object Type: %s", pair.first.c_str());
        for (const auto &obj : pair.second)
        {
            RCLCPP_INFO(this->get_logger(), "  ID: %d, Position: [%.2f, %.2f, %.2f], Topic: %s",
                        obj.message.id, obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z, obj.topic_name.c_str());
            STOD_omniverse_publisher->publish(obj.message);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /STOD", object_map_.size());
}

void CommunicatorNode::printMatrix(const Eigen::Matrix4d &matrix, const std::string &label)
{
    RCLCPP_INFO(this->get_logger(), "%s:", label.c_str());
    for (int i = 0; i < 4; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f, %.3f]",
                    matrix(i, 0), matrix(i, 1), matrix(i, 2), matrix(i, 3));
    }
}

CommunicatorNode::~CommunicatorNode()
{
    // Destructor implementation
    RCLCPP_INFO(this->get_logger(), "CommunicatorNode destroyed");
    robot_monitors_.clear();
    object_map_.clear();
    temp_map.clear();
    odom_topics.clear();
    status_topics.clear();
    object_counts.clear();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CommunicatorNode>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    // Initialize the robot monitors
    node->InitializeRobotMonitor();
    // Start the executor
    executor->spin();
    // Clean up
    executor->remove_node(node);
    rclcpp::shutdown();
    return 0;
}
