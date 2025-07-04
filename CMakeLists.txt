cmake_minimum_required(VERSION 3.5)
project(mission_ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Ament / ROS 2 dependencies ---
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_action      REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(ament_index_cpp    REQUIRED)

# --- Third-party libraries ---
find_package(rml                REQUIRED)
find_package(fsm                REQUIRED)
find_package(ctrl_toolbox       REQUIRED)
find_package(sisl_toolbox       REQUIRED)
find_package(auv_core_helper    REQUIRED)

find_library(GEOGRAPHIC_LIBRARY NAMES Geographic)

# pkg-config for jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# --- Source files shared by controller_node ---
set(MISSION_CTRL_SRCS
  src/mission_ctrl/mission_controller.cpp
  src/mission_ctrl/states/state_base.cpp
  src/mission_ctrl/states/state_init.cpp
  src/mission_ctrl/states/state_latlong.cpp
  src/mission_ctrl/states/state_depth.cpp
  src/mission_ctrl/states/state_homing.cpp
  src/mission_ctrl/states/state_search_object.cpp
  src/mission_ctrl/states/state_cross_gate.cpp
  src/mission_ctrl/states/state_inspect_buoy.cpp
  src/mission_ctrl/states/state_inspect_pipes.cpp
  src/mission_ctrl/states/state_search_buoy_area.cpp
  src/mission_ctrl/states/state_update_localization.cpp
  src/mission_ctrl/states/state_sleep.cpp
)

# --- controller_node executable ---
add_executable(controller_node
  src/mission_controller_node.cpp
  ${MISSION_CTRL_SRCS}
)

target_include_directories(controller_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${auv_core_helper_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

ament_target_dependencies(controller_node
  rclcpp rclcpp_action ament_index_cpp auv_core_helper std_msgs
)

target_link_libraries(controller_node
  config++
  Geographic
  rml
  fsm
  ctrl_toolbox
  sisl_toolbox
  ${JSONCPP_LIBRARIES}
  ${auv_core_helper_LIBRARIES}
)

# --- monitor_node executable ---
add_executable(monitor_node
  src/monitor_node.cpp
  src/system_monitor/system_status_monitor.cpp
)

target_include_directories(monitor_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${auv_core_helper_INCLUDE_DIRS}
)

ament_target_dependencies(monitor_node
  rclcpp rclcpp_action ament_index_cpp auv_core_helper std_msgs
)

target_link_libraries(monitor_node
  Geographic
  ctrl_toolbox
  rml
  config++
  ${auv_core_helper_LIBRARIES}
)

# --- Install targets ---
install(
  TARGETS controller_node monitor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY conf
  DESTINATION share/${PROJECT_NAME}
)

# --- ament package declaration ---
ament_package()
