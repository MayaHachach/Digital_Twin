#include "communication/communicator.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node"), logger_("initial_history"), temp_logger("temp_history")
{
    threshold_ = 0.1; // Allow small floating-point differences
    object_map_ = logger_.loadLastIterationToMap();
    temp_map = temp_logger.loadLastTempToMap();
    PrintSTOD();

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
            auto it = object_map_.find(human_corrected_msg->name);
            //TODO: create a function named isObjectFound and create one for a class and one for a class and id like this one
            if (it == object_map_.end() || human_corrected_msg->id - 1 >= it->second.size())
            {
                // The class name wasn't found in the map
            RCLCPP_INFO(this->get_logger(), "Class %s was not found in the object map", human_corrected_msg->name.c_str());
            return;
        }
        
            // class is found
            auto &object = object_map_.at(human_corrected_msg->name)[human_corrected_msg->id - 1];

            if (object.topic_name != "")
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
                printMatrix(T_corrected, "T_corrected: ");
                printMatrix(T_object.inverse(), "T_object: ");
                printMatrix(odometry_topic->second.T_human_correction, "T_human_correction: ");
                
            }

            RCLCPP_INFO(this->get_logger(), "Human agent suggested a correction to %s %d to position [%.2f, %.2f, %.2f]", 
                human_corrected_msg->name.c_str(), human_corrected_msg->id, human_corrected_msg->pose.position.x, human_corrected_msg->pose.position.y, human_corrected_msg->pose.position.z);
            

            // replace the current pose with the corrected pose
            object.message.pose = human_corrected_msg->pose; // applies for both online and offline objects
            PrintSTOD();
            omniverse_publisher->publish(object.message);
            logger_.logAllObjects(object_map_); });

        for (auto &pair : odom_topics)
    { // fill the odom_msg in odom_topics map
        robot_odom_subscribers_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            pair.first, 10,
            [this, &pair](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                // find the corresponding object
                if (!isObjectInSTOD(pair.second.class_name, pair.second.id))
                    return;
                auto &object = object_map_.at(pair.second.class_name)[pair.second.id - 1];
                object.topic_name = pair.first;

                Eigen::Matrix4d T_odom = poseToTransformation(msg->pose.pose);

                // consider the case of robot shutdown and system shutdown
                if (pair.second.T_global_frame.isIdentity() || T_odom.isIdentity()) // at the initialization of the robot
                {
                    Eigen::Matrix4d T_object = poseToTransformation(object.message.pose);
                    // calculate the transformation of the odom from local frame to global frame
                    pair.second.T_global_frame = T_object * T_odom.inverse();

                    // Reset human correction since it's already applied in the object pose
                    pair.second.T_human_correction = Eigen::Matrix4d::Identity();
                    RCLCPP_INFO(this->get_logger(), "T_global frame was recalculated by T_object * T_odom.inverse() and T_human_correction was reset");
                    printMatrix(T_object, "T_object: ");
                    printMatrix(T_odom.inverse(), "T_odom.inverse(): ");
                    printMatrix(pair.second.T_global_frame, "T_global_frame: ");

                }

                // apply the transformation to the odom msg and store it in the odom_msg
                Eigen::Matrix4d T_correct_odom = pair.second.T_human_correction * pair.second.T_global_frame * T_odom;
                RCLCPP_INFO(this->get_logger(), "T_correct odom was multiplied by human correcrion and global frame");
                RCLCPP_INFO(this->get_logger(), "pair.second.T_human_correction * pair.second.T_global_frame * T_odom");
                printMatrix(pair.second.T_human_correction, "T_human_correction: ");
                printMatrix(pair.second.T_global_frame, "T_global_frame: ");
                printMatrix(T_odom, "T_odom: ");
                printMatrix(T_correct_odom, "T_correct_odom: ");


                pair.second.odom_msg = TransformationToPose(T_correct_odom);
                RCLCPP_INFO(this->get_logger(), "T_correct_odom was filled in pair.second.odom_msg");

                // update the object pose in the STOD when the pose is updated
                if (isPoseEqual(object.message.pose, pair.second.odom_msg.pose.pose))
                {
                    return;
                }
                object.message.pose = pair.second.odom_msg.pose.pose;

                // publish the updated object
                omniverse_publisher->publish(object.message);
                PrintSTOD();
            logger_.logAllObjects(object_map_);
            }));
    }



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
STOD_hololens_publisher = this->create_publisher<customed_interfaces::msg::Object>("/hololensSTOD", 10);
    STOD_omniverse_publisher = this->create_publisher<customed_interfaces::msg::Object>("/omniverseSTOD", 10);
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommunicatorNode>());
    rclcpp::shutdown();
    return 0;
}
