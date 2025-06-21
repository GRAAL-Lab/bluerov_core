cmake_minimum_required(VERSION 3.5)

project(ctrl_station)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add common compile warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  "${CMAKE_SOURCE_DIR}/include/jsoncons/include"
  "${CMAKE_SOURCE_DIR}/include/json_utils"
  "${CMAKE_SOURCE_DIR}/include"
)

# Optional: add /usr/local/include if it exists (avoid hardcoding)
if(EXISTS "/usr/local/include")
  include_directories("/usr/local/include")
endif()

# Link directories (optional and avoided if possible)
if(EXISTS "/usr/local/lib")
  link_directories("/usr/local/lib")
endif()

# Find external libraries
find_library(PAHO_MQTTPP3_LIB paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTT3AS_LIB paho-mqtt3as REQUIRED)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auv_core_helper REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mission_ctrl REQUIRED)
find_package(ctrl_toolbox REQUIRED)

# Define json_utils static library
add_library(json_utils STATIC
  src/json_utils.cpp
)

target_include_directories(json_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/json_utils>"
)

# Executables
add_executable(mqtt_client src/mqtt_client.cpp)
add_executable(terminal src/terminal_node.cpp)
add_executable(cmd_dispatcher src/command_dispatcher_node.cpp)

target_include_directories(cmd_dispatcher PRIVATE
  ${mission_ctrl_INCLUDE_DIRS}
)

# Set compile features explicitly (optional since CMAKE_CXX_STANDARD is set)
target_compile_features(mqtt_client PUBLIC cxx_std_17)
target_compile_features(terminal PUBLIC cxx_std_17)
target_compile_features(cmd_dispatcher PUBLIC cxx_std_17)

# Link libraries to executables
target_link_libraries(mqtt_client
  json_utils
  ${PAHO_MQTTPP3_LIB}
  ${PAHO_MQTT3AS_LIB}
  # Add other libraries here if necessary, e.g. rml
)

target_link_libraries(cmd_dispatcher
  config++
)

# Link dependencies using ament for ROS2 targets
ament_target_dependencies(cmd_dispatcher
  rclcpp
  rclcpp_action
  std_msgs
  auv_core_helper
  ament_index_cpp
  ctrl_toolbox
)

ament_target_dependencies(mqtt_client
  rclcpp
  std_msgs
)

ament_target_dependencies(terminal
  rclcpp
  std_msgs
  ament_index_cpp
)

# Install targets
install(TARGETS
  terminal
  cmd_dispatcher
  mqtt_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY conf/
  DESTINATION share/${PROJECT_NAME}/conf
)

install(PROGRAMS
  scripts/download_rov_data.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)


# Testing and linting (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
