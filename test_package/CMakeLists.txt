cmake_minimum_required(VERSION 3.5)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-std=c++17)

project(test_package)

find_library(PAHO_MQTTPP3_LIB paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTT3AS_LIB paho-mqtt3as REQUIRED)

include_directories("${CMAKE_SOURCE_DIR}/include/jsoncons/include")
include_directories("${CMAKE_SOURCE_DIR}/include/ctrl_toolbox/include/")
include_directories("${CMAKE_SOURCE_DIR}/include/json_utils")
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("/usr/local/include")

link_directories("/usr/local/lib")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auv_core_helper REQUIRED)
find_package(ament_index_cpp REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_library(json_utils STATIC
    src/json_utils.cpp
)

target_include_directories(json_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/json_utils>"
)

# Executable target
add_executable(my_test src/test_node.cpp)
add_executable(my_terminal src/terminal.cpp)
add_executable(mqtt_client src/mqtt_client.cpp)
add_executable(terminal src/terminal_node.cpp)
add_executable(cmd_dispatcher src/command_dispatcher_node.cpp)

target_compile_features(my_terminal PUBLIC cxx_std_17)
target_compile_features(cmd_dispatcher PUBLIC cxx_std_17)

# Link against the json_utils library
target_link_libraries(my_test
  json_utils
)

target_link_libraries(mqtt_client
    json_utils
    ${PAHO_MQTTPP3_LIB}
    ${PAHO_MQTT3AS_LIB}
    rml
    stdc++fs
)

target_link_libraries(my_terminal config++) 
target_link_libraries(cmd_dispatcher config++)

ament_target_dependencies(my_test rclcpp std_msgs auv_core_helper)
ament_target_dependencies(my_terminal rclcpp std_msgs auv_core_helper ament_index_cpp)
ament_target_dependencies(cmd_dispatcher rclcpp std_msgs auv_core_helper ament_index_cpp)
ament_target_dependencies(mqtt_client rclcpp std_msgs)
ament_target_dependencies(terminal rclcpp std_msgs)

install(TARGETS terminal cmd_dispatcher my_test my_terminal mqtt_client DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY conf/
  DESTINATION share/${PROJECT_NAME}/conf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
