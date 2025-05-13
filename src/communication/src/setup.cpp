#include "communication/setup.hpp"

SetupNode::SetupNode() : Node("setup_node"), logger_("initial_history")
{
    initial_stod_subscriber_ = this->create_subscription<customed_interfaces::msg::Object>(
        "/initialSTOD", 10,
        std::bind(&SetupNode::STODExtractionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Setup node initialized and waiting for STOD...");

}

void SetupNode::STODExtractionCallback(const customed_interfaces::msg::Object::SharedPtr msg)
{
    if (msg->name == "End")
    {
        for (auto &pair : object_map_)
        {
            object_counts_[pair.first] = pair.second.size();

            std::sort(pair.second.begin(), pair.second.end(),
                      [](const auto &a, const auto &b)
                      { return a.message.id < b.message.id; });
        }
        std::map<std::string, std::shared_ptr<communication::RobotMonitor>> robotMonitors;
        RCLCPP_INFO(this->get_logger(), "STOD extraction complete. Found %zu objects.", object_map_.size());
        // logger_.logAllObjects(object_map_, robotMonitors);
        logger_.logAllObjects(object_map_);
        GenerateConfigFile();

        RCLCPP_INFO(this->get_logger(), "Setup complete. Please edit the generated config file if needed, then run the launch file.");
        rclcpp::shutdown();
        return;
    }

    if (isObjectInSTOD(msg->name, msg->id))
        return;

    AddNewObject(*msg);
}

void SetupNode::GenerateConfigFile()
{
    try
    {
        auto source_path = std::filesystem::path(std::string(__FILE__));
        auto workspace_src = source_path.parent_path().parent_path().parent_path().parent_path().parent_path();
        auto config_dir = workspace_src / "Digital_Twin" / "src" / "communication" / "config";
        auto yaml_path = config_dir / "robots.yaml";
        auto robot_names_path = config_dir / "robot_names.yaml";

        YAML::Node robot_names = YAML::LoadFile(robot_names_path.string());
        YAML::Node class_list = robot_names["communicator_node"]["ros_parameters"]["robot_classes"];

        std::set<std::string> valid_classes;
        for (const auto &entry : class_list)
            valid_classes.insert(entry.as<std::string>());

        YAML::Node root;
        YAML::Node communicator = root["communicator_node"];
        YAML::Node ros_parameters = communicator["ros_parameters"];
        YAML::Node robots_node = ros_parameters["robots"];

        for (const auto &entry : valid_classes)
        {
            if (object_map_.find(entry) == object_map_.end())
                continue;

            for (const auto &obj : object_map_[entry])
            {
                std::string robot_name = entry + std::to_string(obj.message.id);
                robots_node[robot_name]["type"] = entry;
                robots_node[robot_name]["topics"]["odometry"] = "e.g. /" + robot_name + "/odom";
                robots_node[robot_name]["topics"]["navigation"]["navigation_path"] = "e.g. /" + robot_name + "/transformed_global_plan";
                robots_node[robot_name]["topics"]["navigation"]["navigation_status"] = "e.g. /" + robot_name + "/navigate_to_pose/_action/status";
                robots_node[robot_name]["topics"]["feedback"]["robot_status"] = "e.g. /" + robot_name + "/status";
            }
        }

        std::ofstream fout(yaml_path);
        fout << root;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Generated robots.yaml at: %s", yaml_path.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write robots.yaml: %s", e.what());
    }
}

bool SetupNode::isObjectInSTOD(const std::string &object_class_name, int object_id)
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

DataLogger::object_map_struct SetupNode::AddNewObject(const customed_interfaces::msg::Object &message)
{
    DataLogger::object_map_struct new_object;
    new_object.message = message;

    object_map_[message.name].push_back(new_object);
    RCLCPP_INFO(this->get_logger(), "Added new object: %s with id %d", message.name.c_str(), new_object.message.id);
    
    return new_object;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetupNode>());
    rclcpp::shutdown();
    return 0;
}