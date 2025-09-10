cmake_minimum_required(VERSION 3.10)
project(communication)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customed_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})

add_library(data_logger src/data_logger.cpp)
ament_target_dependencies(data_logger rclcpp ament_index_cpp nlohmann_json customed_interfaces)

target_include_directories(data_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(communicator_node
  src/communicator.cpp
  src/data_logger.cpp
  src/robot_monitor/husky_robot.cpp
  src/robot_monitor/kobuki_robot.cpp
  src/robot_monitor/locobot_robot.cpp
  src/robot_monitor/robot_monitor_factory.cpp
)

ament_target_dependencies(communicator_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  customed_interfaces
  nlohmann_json
  yaml-cpp
  sensor_msgs
  irobot_create_msgs
  husky_msgs
)

target_include_directories(communicator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(communicator_node yaml-cpp)

# add_executable(navigation src/navigation.cpp)
# ament_target_dependencies(navigation rclcpp customed_interfaces geometry_msgs nav_msgs)

# target_include_directories(navigation PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

add_executable(day0_creation src/day0_creation.cpp)
ament_target_dependencies(day0_creation rclcpp std_msgs ament_index_cpp customed_interfaces)

add_executable(request_history_server src/request_history_server.cpp)
ament_target_dependencies(request_history_server rclcpp std_msgs nlohmann_json ament_index_cpp customed_interfaces irobot_create_msgs husky_msgs)

add_executable(request_history_client references/request_history_client.cpp)
ament_target_dependencies(request_history_client rclcpp std_msgs nlohmann_json ament_index_cpp customed_interfaces)

add_executable(husky_manager src/husky_manager.cpp)
ament_target_dependencies(husky_manager rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs customed_interfaces husky_msgs)

add_executable(setup_node src/setup.cpp)
target_link_libraries(setup_node
  data_logger
  yaml-cpp
)
ament_target_dependencies(setup_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  customed_interfaces
  yaml-cpp
)


install(TARGETS
  data_logger
  communicator_node
  day0_creation
  request_history_server
  request_history_client
  husky_manager
  setup_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Header Files
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
# install(
#   DIRECTORY
#   config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install Python script properly
install(
  PROGRAMS
  navigation/navigation.py 
  scripts/dummy_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
