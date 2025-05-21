#include "communication/data_logger.hpp"

DataLogger::DataLogger(const std::string &user_file_name)
{
    // Resolve the path to the JSON file
    auto source_path = std::filesystem::path(__FILE__);
    RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "source path: %s", source_path.c_str());
    auto workspace_src = source_path.parent_path().parent_path().parent_path(); // Gets you to .../src/communication
    RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "workspace path: %s", log_file_path_.c_str());
    auto history_dir = workspace_src / "communication" / "history"; // .../src/communication/history
    log_file_path_ = (history_dir / (user_file_name + ".json")).string();

    RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "log path: %s", log_file_path_.c_str());

    // Initialize the JSON file (create if it doesn't exist)
    initializeJsonFile();
}

// Method to initialize the JSON file
void DataLogger::initializeJsonFile()
{
    std::ifstream file(log_file_path_); // Uses std::ifstream to attempt opening the file at log_file_path_.
    if (!file.good())                   // The good() method checks if the file exists and is accessible. In this if, the file doesn't exist
    {
        // File does not exist; create it
        nlohmann::json initial_data;
        initial_data["Isaac sim data"] = nlohmann::json::object(); // initial_data is a nlohmann::json object, which is a type mann::json library for handling JSON structures in C++.

        std::ofstream new_file(log_file_path_); // std::ofstream opens the file for writing
        if (new_file.is_open())
        {
            new_file << initial_data.dump(4); // Pretty print with 4 spaces
            new_file.close();
            RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "Initialized new JSON log file: %s", log_file_path_.c_str());
        }
        else
        {
            throw std::runtime_error("Failed to create log file.");
        }
    }
}

// Method to log data
void DataLogger::logData(const nlohmann::json &new_entry)
{
    nlohmann::json log_data;

    // Load existing JSON file if available
    try
    {
        std::ifstream file(log_file_path_);
        if (file.is_open())
        {
            file >> log_data;
            file.close();
        }
        else
        {
            log_data["Isaac sim data"] = nlohmann::json::object();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DataLogger"), "Error reading JSON file: %s", e.what());
        log_data["Isaac sim data"] = nlohmann::json::object();
    }

    // Append the new entry to the JSON data
    auto &data = log_data["Isaac sim data"];
    data[std::to_string(data.size())] = new_entry;

    // Save the updated JSON back to the file
    try
    {
        std::ofstream file(log_file_path_);
        if (file.is_open())
        {
            file << log_data.dump(4); // Pretty print with 4 spaces
            file.close();
            // RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "Data logged successfully.");
        }
        else
        {
            throw std::runtime_error("Failed to open log file for writing.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DataLogger"), "Error writing JSON file: %s", e.what());
    }
}

void DataLogger::logAllObjects(std::unordered_map<std::string, vector<DataLogger::object_map_struct>> &objectMap)
{
    // Generate a timestamp
    std::string timestamp = GetTimestamp();

    // Create a JSON entry
    nlohmann::json json_entry;
    json_entry["timestamp"] = timestamp;
    json_entry["objects"] = nlohmann::json::array();

    // Add all objects in the map to the JSON array
    for (const auto &pair : objectMap)
    {
        const auto &objects = pair.second;

        // Add all objects with the same name but different IDs to the JSON array
        for (const auto &obj : objects)
        {
            json_entry["objects"].push_back({{"name", obj.message.name + std::to_string(obj.message.id)},
                                             {"id", obj.message.id},
                                             {"topic_name", obj.topic_name},
                                             {"pose",
                                              {{"position", {obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z}},
                                               {"orientation", {obj.message.pose.orientation.w, obj.message.pose.orientation.x, obj.message.pose.orientation.y, obj.message.pose.orientation.z}}}},
                                             {"scale", {obj.message.scale.x, obj.message.scale.y, obj.message.scale.z}}});
        }
    }

    // Log the data to the JSON file
    logData(json_entry);
    RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Logged data for %zu objects", objectMap.size());
}

void DataLogger::logAllObjects(std::unordered_map<std::string, vector<DataLogger::object_map_struct>> &objectMap, std::map<std::string, std::shared_ptr<communication::RobotMonitor>> &robotMonitors)
{
    // Generate a timestamp
    std::string timestamp = GetTimestamp();

    // Create a JSON entry
    nlohmann::json json_entry;
    json_entry["timestamp"] = timestamp;
    json_entry["objects"] = nlohmann::json::array();

    // Add all objects in the map to the JSON array
    for (const auto &pair : objectMap)
    {
        const auto &objects = pair.second;

        // Add all objects with the same name but different IDs to the JSON array
        for (const auto &obj : objects)
        {
            RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Logging object: %s with id %d", obj.message.name.c_str(), obj.message.id);
            // if(robotMonitors.empty())
            auto robot_name = obj.message.name + std::to_string(obj.message.id);
            std::shared_ptr<communication::RobotMonitor> robot_monitor;
            auto it = robotMonitors.find(robot_name);

            RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Found robot monitor for %s", robot_name.c_str());
            nlohmann::json robot_status;
            nlohmann::json robot_navigation_path;

            if (it != robotMonitors.end())
            {
                robot_monitor = it->second;
                for (const auto &[robot_name, robot_data] : robot_monitor->getStatus().data)
                {
                    robot_status[robot_name] = robot_data;
                    RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Robot status: %s", robot_status.dump().c_str());
                }

                // navigation_path
                const auto &path = robot_monitor->getStatus().navigation_path_data;
                for (const auto &pose_stamped : path.poses)
                {
                    robot_navigation_path["poses"].push_back({{"position", {pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z}},
                                                              {"orientation", {pose_stamped.pose.orientation.w, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z}}});
                }
            }

            else
            {
                RCLCPP_WARN(rclcpp::get_logger("Datalogger"), "No robot monitor found for %s", robot_name.c_str());
            }
            json_entry["objects"].push_back({{"name", obj.message.name + std::to_string(obj.message.id)},
                                             {"id", obj.message.id},
                                             {"topic_name", obj.topic_name},
                                             {"pose",
                                              {{"position", {obj.message.pose.position.x, obj.message.pose.position.y, obj.message.pose.position.z}},
                                               {"orientation", {obj.message.pose.orientation.w, obj.message.pose.orientation.x, obj.message.pose.orientation.y, obj.message.pose.orientation.z}}}},
                                             {"status", robot_status.empty() ? "No status available" : robot_status},
                                             {"navigation_path", robot_navigation_path.empty() ? "No navigation path available" : robot_navigation_path},
                                             {"scale", {obj.message.scale.x, obj.message.scale.y, obj.message.scale.z}}});
        }

        // Log the data to the JSON file
    }
    //add data to the queue to be logged
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_queue_.push(json_entry);
    RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Logged data for %zu objects", objectMap.size());
}

void DataLogger::logTempObjects(std::unordered_map<std::string, vector<customed_interfaces::msg::Object>> &Map)
{
    // Generate a timestamp
    std::string timestamp = GetTimestamp();

    // Create a JSON entry
    nlohmann::json json_entry;
    json_entry["timestamp"] = timestamp;
    json_entry["objects"] = nlohmann::json::array();

    // Add all objects in the map to the JSON array
    for (const auto &pair : Map)
    {
        const auto &objects = pair.second;

        // Add all objects with the same name but different IDs to the JSON array
        for (const auto &obj : objects)
        {
            json_entry["objects"].push_back({{"name", obj.name + std::to_string(obj.id)},
                                             {"id", obj.id},
                                             {"pose",
                                              {{"position", {obj.pose.position.x, obj.pose.position.y, obj.pose.position.z}},
                                               {"orientation", {obj.pose.orientation.w, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z}}}},
                                             {"scale", {obj.scale.x, obj.scale.y, obj.scale.z}}});
        }
    }

    // Log the data to the JSON file
    logData(json_entry);
    RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Logged data for %zu objects", Map.size());
}

// Helper function to generate a timestamp
std::string DataLogger::GetTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time_now), "%Y-%m-%dT%H:%M:%S");
    return timestamp.str();
}

nlohmann::json DataLogger::ROSMessageToJson(const customed_interfaces::msg::Object &msg, const std::string &timestamp)
{
    // Convert the message fields to JSON format
    nlohmann::json json_entry;
    json_entry["timestamp"] = timestamp; // Add timestamp
    json_entry["objects"] = {
        {
            {"name", msg.name}, // Assuming the message has a `name` field
            {"pose",
             {
                 {"position", {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z}},                                    // Assuming message has position
                 {"orientation", {msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w}} // Assuming orientation is a quaternion
             }},
            {"scale", {msg.scale.x, msg.scale.y, msg.scale.z}} // Assuming message has scale
        }};
    return json_entry;
}

geometry_msgs::msg::Pose DataLogger::getLastPose(const std::string &name, int id)
{
    geometry_msgs::msg::Pose pose;
    std::ifstream in_file(this->log_file_path_);
    if (!in_file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("Datalogger"), "Failed to open log file.");
        return pose;
    }

    nlohmann::json json_data;
    in_file >> json_data;

    // Iterate backward to find the most recent entry
    for (auto it = json_data.rbegin(); it != json_data.rend(); ++it)
    {
        for (const auto &obj : (*it)["objects"])
        {
            if (obj["name"].get<std::string>() == name + std::to_string(id))
            {
                pose.position.x = obj["pose"]["position"][0];
                pose.position.y = obj["pose"]["position"][1];
                pose.position.z = obj["pose"]["position"][2];

                pose.orientation.w = obj["pose"]["orientation"][0];
                pose.orientation.x = obj["pose"]["orientation"][1];
                pose.orientation.y = obj["pose"]["orientation"][2];
                pose.orientation.z = obj["pose"]["orientation"][3];
                return pose;
            }
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("Datalogger"), "Pose for %s%d not found in history.", name.c_str(), id);
    return pose;
}

unordered_map<string, vector<DataLogger::object_map_struct>> DataLogger::loadLastIterationToMap()
{
    try
    {
        std::ifstream file(log_file_path_);
        if (!file.good())
        {
            RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "JSON file does not exist. Starting with an empty map.");
            return object_map;
        }

        nlohmann::json log_data;
        file >> log_data;

        // Check if the JSON file is empty or malformed
        if (log_data["Isaac sim data"].empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "JSON file is empty. Starting with an empty map.");
            return object_map;
        }

        // Get the last entry
        auto last_entry = *(log_data["Isaac sim data"].rbegin());

        // Parse the objects into the map
        for (const auto &obj : last_entry["objects"])
        {
            object_map_struct object_msg;
            object_msg.message.id = obj["id"];
            // object_msg.message.name = obj["name"];
            object_msg.message.name = removeIDFromName(obj["name"], object_msg.message.id);
            object_msg.topic_name = obj["topic_name"];
            object_msg.message.pose.position.x = obj["pose"]["position"][0];
            object_msg.message.pose.position.y = obj["pose"]["position"][1];
            object_msg.message.pose.position.z = obj["pose"]["position"][2];
            object_msg.message.pose.orientation.w = obj["pose"]["orientation"][0];
            object_msg.message.pose.orientation.x = obj["pose"]["orientation"][1];
            object_msg.message.pose.orientation.y = obj["pose"]["orientation"][2];
            object_msg.message.pose.orientation.z = obj["pose"]["orientation"][3];
            object_msg.message.scale.x = obj["scale"][0];
            object_msg.message.scale.y = obj["scale"][1];
            object_msg.message.scale.z = obj["scale"][2];

            object_map[object_msg.message.name].push_back(object_msg);
        }

        RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Initialized map with %zu objects from JSON.", object_map.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Datalogger"), "Error loading JSON file: %s", e.what());
    }

    return object_map;
}

unordered_map<string, vector<customed_interfaces::msg::Object>> DataLogger::loadLastTempToMap()
{
    try
    {
        std::ifstream file(log_file_path_);
        if (!file.good())
        {
            RCLCPP_INFO(rclcpp::get_logger("DataLogger"), "Temp file does not exist.");
            return temp_map;
        }

        nlohmann::json log_data;
        file >> log_data;

        // Check if the JSON file is empty or malformed
        if (log_data["Isaac sim data"].empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "JSON file is empty. Starting with an empty map.");
            return temp_map;
        }

        // Get the last entry
        auto last_entry = *(log_data["Isaac sim data"].rbegin());

        // Parse the objects into the map
        for (const auto &obj : last_entry["objects"])
        {
            customed_interfaces::msg::Object temp_msg;
            temp_msg.id = obj["id"];
            // temp_msg.name = obj["name"];
            temp_msg.name = removeIDFromName(obj["name"], temp_msg.id);
            temp_msg.pose.position.x = obj["pose"]["position"][0];
            temp_msg.pose.position.y = obj["pose"]["position"][1];
            temp_msg.pose.position.z = obj["pose"]["position"][2];
            temp_msg.pose.orientation.w = obj["pose"]["orientation"][0];
            temp_msg.pose.orientation.x = obj["pose"]["orientation"][1];
            temp_msg.pose.orientation.y = obj["pose"]["orientation"][2];
            temp_msg.pose.orientation.z = obj["pose"]["orientation"][3];
            temp_msg.scale.x = obj["scale"][0];
            temp_msg.scale.y = obj["scale"][1];
            temp_msg.scale.z = obj["scale"][2];

            temp_map[temp_msg.name].push_back(temp_msg);
        }

        RCLCPP_INFO(rclcpp::get_logger("Datalogger"), "Initialized temp map with %zu objects from JSON.", temp_map.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Datalogger"), "Error loading JSON file: %s", e.what());
    }

    return temp_map;
}

// Function to remove the numeric ID from the end of a name
std::string DataLogger::removeIDFromName(const std::string &name, int id)
{
    std::string idStr = std::to_string(id); // Convert ID to string
    size_t idLength = idStr.length();       // Get length of ID string

    // Check if name ends with the ID string
    if (name.length() >= idLength && name.substr(name.length() - idLength) == idStr)
    {
        return name.substr(0, name.length() - idLength); // Remove ID from name
    }

    return name; // Return original name if ID not found at the end
}

void DataLogger::startLoggingThread()
{
    keep_logging_ = true;
    logging_thread_ = std::thread([this]()
                                  {
        while (keep_logging_) {
            nlohmann::json entry;
            {
                std::unique_lock<std::mutex> lock(log_mutex_);
                if (!log_queue_.empty()) {
                    entry = log_queue_.front();
                    log_queue_.pop();
                } else {
                    lock.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
            }
            this->logData(entry);
        } });
}

void DataLogger::stopLoggingThread()
{
    keep_logging_ = false;
    if (logging_thread_.joinable())
    {
        logging_thread_.join();
    }
}
