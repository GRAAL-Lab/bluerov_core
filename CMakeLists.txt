cmake_minimum_required(VERSION 3.5)
project(mission_ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD      17 CACHE STRING "C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE        RelWithDebInfo CACHE STRING "Build type")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Ament / ROS 2 dependencies ---
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(ament_index_cpp    REQUIRED)

# --- Third-party libraries ---
find_package(rml                REQUIRED)
find_package(fsm                REQUIRED)
find_package(ctrl_toolbox      REQUIRED)
find_package(sisl_toolbox       REQUIRED)
find_package(auv_core_helper    REQUIRED)

# pkg-config for jsoncpp
find_package(PkgConfig          REQUIRED)
pkg_check_modules(JSONCPP       REQUIRED jsoncpp)

# --- Library target ---
set(MISSION_CTRL_SRCS
  src/mission_ctrl/mission_controller.cpp
  src/mission_ctrl/states/state_base.cpp
  src/mission_ctrl/states/state_init.cpp
  src/mission_ctrl/states/state_latlong.cpp
  src/mission_ctrl/states/state_homing.cpp
  src/mission_ctrl/states/state_search_object.cpp
  src/mission_ctrl/states/state_cross_gate.cpp
  src/mission_ctrl/states/state_inspect_buoy.cpp
  src/mission_ctrl/states/state_inspect_pipes.cpp
  src/mission_ctrl/states/state_search_buoy_area.cpp

)
# --- Library target ---
add_library(mission_ctrl_lib ${MISSION_CTRL_SRCS})

target_include_directories(mission_ctrl_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${auv_core_helper_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

ament_target_dependencies(mission_ctrl_lib
  rclcpp std_msgs ament_index_cpp auv_core_helper
)

target_link_libraries(mission_ctrl_lib
  config++
  Geographic
  rml
  fsm
  ctrl_toolbox
  sisl_toolbox
  ${JSONCPP_LIBRARIES}
  ${auv_core_helper_LIBRARIES}
)

# --- Executable target ---
add_executable(controller_node src/mission_controller_node.cpp)

target_include_directories(controller_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${auv_core_helper_INCLUDE_DIRS}
)

ament_target_dependencies(controller_node
  rclcpp ament_index_cpp auv_core_helper
)

target_link_libraries(controller_node
  mission_ctrl_lib
  rml
  ${auv_core_helper_LIBRARIES}
)


# --- Install rules ---
# export both library and executable
install(
  TARGETS mission_ctrl_lib controller_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# install non-code resources
install(
  DIRECTORY conf
  DESTINATION share/${PROJECT_NAME}
)

# export include dirs & libraries for downstream
ament_export_include_directories(include)
ament_export_libraries(mission_ctrl_lib)

ament_package()
