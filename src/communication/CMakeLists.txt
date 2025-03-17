cmake_minimum_required(VERSION 3.8)
project(communication)

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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(data_logger src/data_logger.cpp)
ament_target_dependencies(data_logger rclcpp ament_index_cpp nlohmann_json customed_interfaces)

target_include_directories(data_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(communicator src/communicator.cpp)
target_link_libraries(communicator data_logger)
ament_target_dependencies(communicator rclcpp nlohmann_json customed_interfaces geometry_msgs nav_msgs)

target_include_directories(communicator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# add_executable(navigation src/navigation.cpp)
# ament_target_dependencies(navigation rclcpp customed_interfaces geometry_msgs nav_msgs)

# target_include_directories(navigation PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

add_executable(request_history_server src/request_history_server.cpp)
ament_target_dependencies(request_history_server rclcpp std_msgs nlohmann_json ament_index_cpp customed_interfaces)

add_executable(request_STOD_server src/request_STOD_server.cpp)
target_link_libraries(request_STOD_server data_logger)
ament_target_dependencies(request_STOD_server rclcpp std_msgs nlohmann_json ament_index_cpp customed_interfaces)

add_executable(request_history_client src/request_history_client.cpp)
ament_target_dependencies(request_history_client rclcpp std_msgs nlohmann_json ament_index_cpp customed_interfaces)

add_executable(odom_alignment src/odom_alignment.cpp)
ament_target_dependencies(odom_alignment rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  data_logger
  communicator
  request_history_server
  request_STOD_server
  request_history_client
  odom_alignment
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

# Install Python script properly
install(
  PROGRAMS
  navigation/navigation.py 
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
