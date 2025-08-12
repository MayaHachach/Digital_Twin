#include "communication/communicator.hpp"
#include <filesystem>
#include <cstring>

CommunicatorNode::CommunicatorNode() : Node("communicator_node"), logger_("initial_history"), temp_logger("temp_history")
{
    logger_.startLoggingThread();
    threshold_ = 0.1; // Allow small floating-point differences
    object_map_ = logger_.loadLastIterationToMap();
    temp_map = temp_logger.loadLastTempToMap();

    fillObjectCounts();
    fillOdomTopics();
    PrintSTOD();

    InitializePublishers();
    InitializeSubscribers();
    InitializeServices();

    hololens_connection_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&CommunicatorNode::isHololensConnected, this));
}

// Subscription Callbacks
void CommunicatorNode::objectUpdate(const customed_interfaces::msg::Object::SharedPtr msg)
{
    // // Check if the message is the same as the last one
    // if (last_message_ != nullptr && isMessageEqual(*msg, *last_message_))
    // {
    //     RCLCPP_INFO(this->get_logger(), "duplicate message");
    //     return;
    // }

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
        logger_.logAllObjects(object_map_, robot_monitors_);
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
        logger_.logAllObjects(object_map_, robot_monitors_);
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

    bool recalc = topicInfo.T_global_frame.isIdentity() || isOdomReset(topic_name, msg->header.stamp) || topicInfo.recalculate_transformation;

    if (recalc)
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

    if (!recalc && isPoseEqual(object.message.pose, topicInfo.odom_msg.pose.pose))
        return;

    object.message.pose = topicInfo.odom_msg.pose.pose;

    omniverse_publisher->publish(object.message);
    logger_.logAllObjects(object_map_, robot_monitors_);
}

void CommunicatorNode::navigationPathCallback(CommunicatorNode::navigation_struct &topicInfo, const nav_msgs::msg::Path::SharedPtr msg)
{
    // auto &topicInfo = navigation_topics[topic_name];
    auto topic_name = topicInfo.navigation_path_topic_name;
    RCLCPP_INFO(this->get_logger(), "Subscribed to navigation topic: %s", topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "%s robot", topicInfo.class_name.c_str());

    if (!isObjectInSTOD(topicInfo.class_name, topicInfo.id))
    {
        RCLCPP_INFO(this->get_logger(), "%s %d was not found in STOD for path update", topicInfo.class_name.c_str(), topicInfo.id);
        return;
    }

    std::string robot = topicInfo.class_name + std::to_string(topicInfo.id);

    nav_msgs::msg::Path transformed_path;
    transformed_path.header.stamp = msg->header.stamp;
    transformed_path.header.frame_id = robot;

    RCLCPP_WARN(this->get_logger(), "robot: %s", robot.c_str());

    // //Eigen::Matrix4d T_odom_QR = odom_topics.find(object_map_[topicInfo.class_name][topicInfo.id - 1].topic_name)->second.T_global_frame;
    auto &odom_topic = object_map_.at(topicInfo.class_name)[topicInfo.id - 1].topic_name;

    if (odom_topic.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Topic name is empty for object %s %d", topicInfo.class_name.c_str(), topicInfo.id);
        return;
    }

    // auto it = odom_topics.find(odom_topic);
    // Eigen::Matrix4d T_odom_QR = it->second.T_global_frame;
    auto &odom_info = odom_topics.at(odom_topic);

    // Recalculate transformation only when marked
    if (odom_info.recalculate_transformation || odom_info.T_global_frame.isIdentity())
    {
        if (!isObjectInSTOD(topicInfo.class_name, topicInfo.id))
        {
            RCLCPP_WARN(this->get_logger(), "Object not found in STOD during transformation update.");
            return;
        }

        auto &object = object_map_.at(topicInfo.class_name)[topicInfo.id - 1];
        Eigen::Matrix4d T_object = poseToTransformation(object.message.pose);
        Eigen::Matrix4d T_odom = poseToTransformation(odom_info.odom_msg.pose.pose);
        odom_info.T_global_frame = T_object * T_odom.inverse();
        odom_info.recalculate_transformation = false;

        RCLCPP_INFO(this->get_logger(), "Updated T_global_frame for %s %d", topicInfo.class_name.c_str(), topicInfo.id);
        printMatrix(odom_info.T_global_frame, "T_global_frame");
    }

    Eigen::Matrix4d T_odom_QR = odom_info.T_global_frame;

    RCLCPP_WARN(this->get_logger(), "Path is not cleared, using the last path");
    for (auto &pose : msg->poses)
    {
        Eigen::Matrix4d T_path_odom = poseToTransformation(pose.pose);
        Eigen::Matrix4d T_path_QR = T_odom_QR * T_path_odom;

        RCLCPP_WARN(this->get_logger(), "New pose for %s", robot.c_str());
        // Convert back to PoseStamped
        geometry_msgs::msg::PoseStamped transformed_pose_stamped;
        transformed_pose_stamped.pose = TransformationToPose(T_path_QR).pose.pose;

        transformed_path.poses.push_back(transformed_pose_stamped);
    }

    // publish navigation path
    navigation_path_publisher->publish(transformed_path);

    // save navigation path to the monitoring class
    robot_monitors_[robot]->setNavigationPath(transformed_path);
    logger_.logAllObjects(object_map_, robot_monitors_);
}

void CommunicatorNode::navigationStatusCallback(CommunicatorNode::navigation_struct &nav_data, const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Navigation status callback received for topic: %s", nav_data.navigation_path_topic_name.c_str());
    std::string robot = nav_data.class_name + std::to_string(nav_data.id);

    for (const auto &goal_status : msg->status_list)
    {
        auto status_code = goal_status.status;
        RCLCPP_INFO(this->get_logger(), "Navigation status: %d", status_code);
        std::string uuid = uuid_to_string(goal_status.goal_info.goal_id.uuid);

        if (handled_goals_.find(uuid) != handled_goals_.end())
        {
            continue;
        }

        if (status_code == 4 || status_code == 5 || status_code == 6)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Goal %s succeeded!", uuid.c_str());
            handled_goals_.insert(uuid);
            nav_msgs::msg::Path transformed_path;
            transformed_path.header.frame_id = robot;
            transformed_path.poses.clear();
            navigation_path_publisher->publish(transformed_path);
        }
    }
}

void CommunicatorNode::tempResponseCallback(const customed_interfaces::msg::Temp::SharedPtr temp_response_msg)
{
    // if (previous_temp_message != nullptr && isMessageEqual(*temp_response_msg, *previous_temp_message))
    // {
    //     RCLCPP_INFO(this->get_logger(), "duplicate temp message");
    //     return;
    // }
    // previous_temp_message = temp_response_msg;

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

            logger_.logAllObjects(object_map_, robot_monitors_);
            temp_logger.logTempObjects(temp_map);

            break;
        }
    }

    // publish temp object counts
    customed_interfaces::msg::Temp temp_message;
    temp_message.name = temp_response_msg->name;
    temp_message.number = temp_map[temp_response_msg->name].size();
    temp_count_publisher->publish(temp_message);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
    logger_.logAllObjects(object_map_, robot_monitors_);
}

// Service Handlers
void CommunicatorNode::handleRequest(const std::shared_ptr<customed_interfaces::srv::RequestSTOD::Request> request_,
                                     std::shared_ptr<customed_interfaces::srv::RequestSTOD::Response> response_)
{
    std::lock_guard<std::mutex> lock(request_mutex_); // While the callback function is running, no other thread can access variables.

    if (request_->agent_name == "omniverse")
    {
        publishOmniverseSTOD();
        // request_processed_ = true; // Mark as processed
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

bool CommunicatorNode::publishSTODCategory(const std::string &category)
{
    auto it = object_map_.find(category);
    if (it == object_map_.end() || it->second.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Requested category '%s' not found in STOD!", category.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing requested category '%s' to /hololensSTOD...", category.c_str());

    for (const auto &obj : it->second)
    {
        category_hololens_publisher->publish(obj.message);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // small delay
    }
    return true;
}

void CommunicatorNode::handleRequestCategory(const std::shared_ptr<customed_interfaces::srv::RequestCategory::Request> request,
                                             std::shared_ptr<customed_interfaces::srv::RequestCategory::Response> response)
{
    std::lock_guard<std::mutex> lock(request_mutex_);

    std::string category = request->category;

    // ✅ Check if category exists in the map
    bool publish_result = publishSTODCategory(category);

    response->success = publish_result;
}

void CommunicatorNode::handleEditObjectRequest(const std::shared_ptr<customed_interfaces::srv::EditObjects::Request> request,
                                               std::shared_ptr<customed_interfaces::srv::EditObjects::Response> response)
{
    std::lock_guard<std::mutex> lock(request_mutex_);

    auto &message = request->object; // Object msg from request

    if (request->function == "add")
    {
        RCLCPP_INFO(this->get_logger(), "Adding object: %s", request->object.name.c_str());

        if (message.id > 0 && isObjectInSTOD(message.name, message.id))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Cannot add %s %d, an object with this ID already exists!",
                         message.name.c_str(), message.id);

            response->success = false;
            response->message = "Object already exists in STOD";
            return;
        }

        // ✅ If offline (id == 0), assign next sequential ID
        auto new_message = message;
        if (message.id == 0)
        {
            new_message.id = object_map_[message.name].size() + 1;
            RCLCPP_WARN(this->get_logger(),
                        "Offline object detected. Auto-assigned ID: %d",
                        new_message.id);
        }

        // ✅ Add new object safely
        AddNewObject(new_message);
        object_counts[new_message.name] = object_map_[new_message.name].size();
        RCLCPP_INFO(this->get_logger(), "object_counts[%s] = %d",
                    new_message.name.c_str(), object_counts[new_message.name]);

        bool publish_result = publishSTODCategory(new_message.name);

        if (temp_map.find(message.name) != temp_map.end() && !temp_map[message.name].empty())
        {
            RCLCPP_INFO(this->get_logger(), "Publishing objects to Hololens on /objectLocations for temp edit");

            for (const auto &object : object_map_[new_message.name])
            {
                object_locations_publisher->publish(object.message);
            }
        }

        if (!publish_result)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to publish new object category: %s", new_message.name.c_str());
            response->success = false;
            response->message = "Failed to publish new object category";
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ Successfully added new object: %s %d",
                    new_message.name.c_str(), new_message.id);

        response->success = true;
        response->message = "Object added successfully";
        total_object_count += 1; // Increase total object count

        RCLCPP_INFO(this->get_logger(), "Total object count is now %d", total_object_count);
        logger_.setTotalObjectCount(total_object_count); // Only when total changes
        logger_.logAllObjects(object_map_, robot_monitors_);
    }
    else if (request->function == "delete")
    {
        RCLCPP_INFO(this->get_logger(), "Deleting object: %s", request->object.name.c_str());

        auto &object_vector = object_map_[message.name];

        auto it = std::find_if(object_vector.begin(), object_vector.end(),
                               [&message](const DataLogger::object_map_struct &obj)
                               {
                                   return obj.message.id == message.id;
                               });
        int id = message.id;
        customed_interfaces::msg::Object deleted_object;
        if (it != object_vector.end())
        {
            // overwrite the object with the last one
            if (it + 1 != object_vector.end())
            {
                deleted_object = object_vector.back().message; // save the last object
                RCLCPP_INFO(this->get_logger(), "Overwriting object %s %d with the last one in the vector", it->message.name.c_str(), it->message.id);
                *it = object_vector.back();                      // copy the last element to the current position
                it->message.id = id;                             // keep the ID of the deleted object
                object_vector.pop_back();                        // remove the last element
                omniverse_publisher->publish(it->message);       // publish the updated object
                deleted_object.name = "-" + deleted_object.name; // mark the object as deleted
                omniverse_publisher->publish(deleted_object);    // publish the deleted object
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Removing the last object %s %d", it->message.name.c_str(), it->message.id);
                // If it's the last element
                // Just remove it, no need to overwrite
                object_vector.pop_back();
                it->message.name = "-" + it->message.name; // mark the object as deleted
                omniverse_publisher->publish(it->message); // publish the updated object
            }

            RCLCPP_INFO(this->get_logger(), "✅ Successfully deleted object: %s %d",
                        message.name.c_str(), message.id);

            total_object_count -= 1; // Decrease total object count
            RCLCPP_INFO(this->get_logger(), "Total object count is now %d", total_object_count);
            logger_.setTotalObjectCount(total_object_count); // Only when total changes
            // Update object counts
            object_counts[message.name] = object_vector.size();
            RCLCPP_INFO(this->get_logger(), "object_counts[%s] = %d",
                        message.name.c_str(), object_counts[message.name]);
            // Log and publish
            logger_.logAllObjects(object_map_, robot_monitors_);
            bool publish_result = publishSTODCategory(message.name);

            if (temp_map.find(message.name) != temp_map.end() && !temp_map[message.name].empty())
            {
                RCLCPP_INFO(this->get_logger(), "Publishing objects to Hololens on /objectLocations for temp edit");
                for (const auto &object : object_map_[message.name])
                {
                    object_locations_publisher->publish(object.message);
                }
            }

            if (!publish_result)
            {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to publish updated object category: %s", message.name.c_str());
                response->success = false;
                response->message = "Failed to publish updated object category";
                return;
            }
            response->success = true;
            response->message = "Object deleted successfully";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to delete object: %s %d, not found",
                         message.name.c_str(), message.id);
            response->success = false;
            response->message = "Object not found in STOD";
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown function: %s", request->function.c_str());
        response->success = false;
        response->message = "Unknown function";
        return;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Methods
void CommunicatorNode::InitializePublishers()
{
    omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseObject", 10);
    object_locations_publisher = this->create_publisher<customed_interfaces::msg::Object>("/objectLocations", 10);
    STOD_hololens_publisher = this->create_publisher<customed_interfaces::msg::Object>("/hololensSTOD", 10);
    category_hololens_publisher = this->create_publisher<customed_interfaces::msg::Object>("/categorySTOD", 10);
    STOD_omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseSTOD", 10);
    temp_count_publisher = this->create_publisher<customed_interfaces::msg::Temp>("/tempCount", 10);
    navigation_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/navigation_path", 10);
}

void CommunicatorNode::InitializeSubscribers()
{
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    hololens_object_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/hololensObject", 10,
        std::bind(&CommunicatorNode::objectUpdate, this, std::placeholders::_1));

    human_correction_subscriber = this->create_subscription<customed_interfaces::msg::Object>(
        "/humanCorrection", 10, std::bind(&CommunicatorNode::humanCorrectionCallback, this, std::placeholders::_1));

    temp_response_subscriber = this->create_subscription<customed_interfaces::msg::Temp>(
        "/tempResponse", 10, std::bind(&CommunicatorNode::tempResponseCallback, this, std::placeholders::_1));

    for (auto &pair : odom_topics)
    {
        const std::string &topic_name = pair.first;

        robot_odom_subscribers_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, qos_profile,
            [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->robotOdomCallback(topic_name, msg);
            }));
    }

    for (auto &[robot_name, nav_data] : navigation_topics)
    {
        const std::string &path_topic = nav_data.navigation_path_topic_name;
        const std::string &status_topic = nav_data.navigation_status_topic_name;
        if (!path_topic.empty())
        {
            navigation_path_subscribers_.push_back(this->create_subscription<nav_msgs::msg::Path>(
                path_topic, qos_profile,
                [this, &nav_data](const nav_msgs::msg::Path::SharedPtr msg)
                {
                    this->navigationPathCallback(nav_data, msg);
                }));
        }
        if (!status_topic.empty())
        {
            navigation_status_subscribers_.push_back(this->create_subscription<action_msgs::msg::GoalStatusArray>(
                status_topic, qos_profile,
                [this, &nav_data](const action_msgs::msg::GoalStatusArray::SharedPtr msg)
                {
                    this->navigationStatusCallback(nav_data, msg);
                }));
        }
    }
}

void CommunicatorNode::InitializeServices()
{
    STOD_service = this->create_service<customed_interfaces::srv::RequestSTOD>(
        "request_STOD",
        std::bind(&CommunicatorNode::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

    request_category_service_ = this->create_service<customed_interfaces::srv::RequestCategory>(
        "request_category",
        std::bind(&CommunicatorNode::handleRequestCategory, this, std::placeholders::_1, std::placeholders::_2));

    edit_object_service_ = this->create_service<customed_interfaces::srv::EditObjects>(
        "edit_objects",
        std::bind(&CommunicatorNode::handleEditObjectRequest, this, std::placeholders::_1, std::placeholders::_2));
}

std::string CommunicatorNode::uuid_to_string(const std::array<uint8_t, 16> &uuid)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (const auto &byte : uuid)
    {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    return ss.str();
}

void CommunicatorNode::InitializeRobotMonitor()
{
    // Load robot configurations from YAML file
    // std::string config_path = this->declare_parameter("config_path", "config/robots.yaml");
    auto source_path = std::filesystem::path(std::string(__FILE__));
    auto workspace_src = source_path.parent_path().parent_path().parent_path().parent_path().parent_path();
    auto config_dir = workspace_src / "Digital_Twin" / "src" / "communication" / "config";
    auto config_path = config_dir / "robots.yaml";

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
                robot_monitors_[robot_id] = monitor;
                // RCLCPP_INFO(this->get_logger(), "Created monitor for robot %s of type %s",
                // robot_id.c_str(), config.type.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create monitor for robot %s",
                             robot_id.c_str());
            }
            // RCLCPP_INFO(this->get_logger(), "--------------------------------------");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error initializing robot monitors: %s", e.what());
    }
}

std::map<std::string, RobotConfig> CommunicatorNode::loadRobotConfigs(const std::string &filename)
{
    std::map<std::string, RobotConfig> configs;

    try
    {
        YAML::Node config = YAML::LoadFile(filename);
        // RCLCPP_INFO(this->get_logger(), "Loading robots from communicator_node");
        // RCLCPP_INFO(this->get_logger(), "--------------------------------------");
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

void CommunicatorNode::fillObjectCounts()
{
    if (object_map_.empty())
        return;

    for (auto &pair : object_map_)
    {
        int count = pair.second.size();
        object_counts[pair.first] = pair.second.size();
        total_object_count += count;
        RCLCPP_INFO(this->get_logger(), "Object count of class %s is %d", pair.first.c_str(), object_counts[pair.first]);
    }
    // object_counts["total"] = total_object_count;
    RCLCPP_INFO(this->get_logger(), "Total object count is %d", total_object_count);
}

void CommunicatorNode::fillOdomTopics()
{
    // TODO: what happens if the STOD is empty
    if (object_map_.empty())
    {
        // RCLCPP_WARN(this->get_logger(), "Object map is empty, cannot fill odom topics.");
        return;
    }

    // Check if odom_topics is already initialized
    //! does it work ? this block is callled once in the constructor
    if (!odom_topics.empty())
    {
        RCLCPP_INFO(this->get_logger(), "odom_topics is already initialized, assigning topic names to object_map.");
    }

    else
    {

        try
        {
            // Locate config file path
            auto source_path = std::filesystem::path(std::string(__FILE__));
            auto workspace_src = source_path.parent_path().parent_path().parent_path().parent_path().parent_path();
            auto config_dir = workspace_src / "Digital_Twin" / "src" / "communication" / "config";
            auto yaml_path = config_dir / "robots.yaml";

            // Load the YAML
            YAML::Node root = YAML::LoadFile(yaml_path.string());
            YAML::Node robots_node = root["communicator_node"]["ros_parameters"]["robots"];

            for (const auto &robot_entry : robots_node)
            {
                std::string robot_name = robot_entry.first.as<std::string>();
                std::string robot_type = robot_entry.second["type"].as<std::string>();
                std::string number_part = std::string(robot_name.begin() + robot_type.length(), robot_name.end());
                int id = std::stoi(number_part);

                if (robot_entry.second["topics"] && robot_entry.second["topics"]["odometry"])
                {
                    std::string odom_topic = robot_entry.second["topics"]["odometry"].as<std::string>();

                    // Skip default placeholders like "e.g."
                    if (!odom_topic.empty() && odom_topic.find("e.g.") == std::string::npos)
                    {
                        std::string number_part = std::string(robot_name.begin() + robot_type.length(), robot_name.end());
                        int id = std::stoi(number_part);

                        odom_topics[odom_topic] = topics_struct{robot_type, id};
                        RCLCPP_INFO(this->get_logger(), "Loaded odometry topic from config: %s -> (%s, %d)",
                                    odom_topic.c_str(), robot_type.c_str(), id);
                    }
                }

                navigation_struct nav_data;
                nav_data.class_name = robot_type;
                nav_data.id = id;

                if (robot_entry.second["topics"] && robot_entry.second["topics"]["navigation"]["navigation_path"])
                {
                    std::string navigation_path_topic = robot_entry.second["topics"]["navigation"]["navigation_path"].as<std::string>();

                    // Skip default placeholders like "e.g."
                    if (!navigation_path_topic.empty() && navigation_path_topic.find("e.g.") == std::string::npos)
                    {

                        nav_data.navigation_path_topic_name = navigation_path_topic;

                        // navigation_topics[navigation_path_topic] = nav_data;
                        RCLCPP_INFO(this->get_logger(), "Loaded navigation path topic from config: %s -> (%s, %d)",
                                    navigation_path_topic.c_str(), robot_type.c_str(), id);
                    }
                }

                if (robot_entry.second["topics"] && robot_entry.second["topics"]["navigation"]["navigation_status"])
                {
                    std::string navigation_status_topic = robot_entry.second["topics"]["navigation"]["navigation_status"].as<std::string>();

                    // Skip default placeholders like "e.g."
                    if (!navigation_status_topic.empty() && navigation_status_topic.find("e.g.") == std::string::npos)
                    {

                        nav_data.navigation_status_topic_name = navigation_status_topic;

                        // navigation_topics[navigation_path_topic] = nav_data;
                        RCLCPP_INFO(this->get_logger(), "Loaded navigation status topic from config: %s -> (%s, %d)",
                                    navigation_status_topic.c_str(), robot_type.c_str(), id);
                    }
                }

                navigation_topics[robot_name] = nav_data;
            }

            if (odom_topics.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid odometry topics found in robots.yaml");
            }

            if (navigation_topics.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid navigation path topics found in robots.yaml");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize odom_topics from robots.yaml: %s", e.what());
        }
    }

    // After loading odom_topics, assign topic names to object_map_
    for (auto &odom : odom_topics)
    {
        if (!isObjectInSTOD(odom.second.class_name, odom.second.id))
        {
            RCLCPP_WARN(this->get_logger(), "Object %s with ID %d not found in object_map_ for topic %s",
                        odom.second.class_name.c_str(), odom.second.id, odom.first.c_str());
            continue;
        }
        auto &robot = object_map_.at(odom.second.class_name)[odom.second.id - 1];
        robot.topic_name = odom.first;
        RCLCPP_INFO(this->get_logger(), "Assigned topic name %s to object of class %s with id %d", odom.first.c_str(), odom.second.class_name.c_str(), odom.second.id);
    }
}

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

    if (message.id == 0)
    {
        // If id is 0, assign a new id based on the current size of the object_map_[message.name]
        new_object.message.id = object_map_[message.name].size() + 1;
    }

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

// Add this in CommunicatorNode class or a helper namespace
bool CommunicatorNode::isOdomReset(const std::string &topic_name, const builtin_interfaces::msg::Time &current_stamp)
{
    static std::unordered_map<std::string, builtin_interfaces::msg::Time> last_stamps;

    auto it = last_stamps.find(topic_name);

    // First message received for this topic
    if (it == last_stamps.end())
    {
        last_stamps[topic_name] = current_stamp;
        return false;
    }

    const auto &last_stamp = it->second;

    // Check if time has reset or is earlier than previous timestamp
    if ((current_stamp.sec < last_stamp.sec) ||
        (current_stamp.sec == last_stamp.sec && current_stamp.nanosec < last_stamp.nanosec))
    {
        RCLCPP_INFO(this->get_logger(), "Timestamp jumped backwards on topic %s", topic_name.c_str());
        last_stamps[topic_name] = current_stamp;
        return true;
    }

    // Normal case, update timestamp and return false
    last_stamps[topic_name] = current_stamp;
    return false;
}

void CommunicatorNode::isHololensConnected()
{
    size_t pub_count = hololens_object_subscriber->get_publisher_count();

    if (pub_count > 0 && !hololens_connected)
    {
        hololens_connected = true;
        RCLCPP_INFO(this->get_logger(), "✅ HoloLens connected.");
    }
    else if (pub_count == 0 && hololens_connected)
    {
        hololens_connected = false;
        RCLCPP_WARN(this->get_logger(), "❌ HoloLens disconnected. Sending empty pose...");

        customed_interfaces::msg::Object hololens_msg;
        hololens_msg.name = "Hololens";
        hololens_msg.id = 1;

        hololens_msg.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        hololens_msg.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        hololens_msg.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        hololens_msg.pose.orientation.x = 0.0;
        hololens_msg.pose.orientation.y = 0.0;
        hololens_msg.pose.orientation.z = 0.0;
        hololens_msg.pose.orientation.w = 1.0;

        omniverse_publisher->publish(hololens_msg);
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
    RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /hololensSTOD", object_map_.size());
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    RCLCPP_INFO(this->get_logger(), "Published %ld object_classes to /omniverseSTOD", object_map_.size());
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
    object_map_.clear();
    temp_map.clear();
    odom_topics.clear();
    object_counts.clear();
    // status_topics.clear();
    robot_monitors_.clear();
    logger_.stopLoggingThread();
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
