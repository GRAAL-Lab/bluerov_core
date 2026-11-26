cmake_minimum_required(VERSION 3.8)
project(bridge)

option(BRIDGE_ENABLE_CAMERA "Build the optional GStreamer camera bridge" OFF)
option(BRIDGE_ENABLE_VIDEO_VIEWER "Build and install the optional UDP video viewer" OFF)

################################################################################
# Set compiler flags
################################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic -Wno-address-of-packed-member)



################################################################################
# Find packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bleurov_core_helper REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavlink REQUIRED)

if(BRIDGE_ENABLE_CAMERA OR BRIDGE_ENABLE_VIDEO_VIEWER)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
    gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()

# Prefer shared libraries over static ones
set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${mavlink_INCLUDE_DIRS}
)


################################################################################
# Source / Executable definitions
################################################################################
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

add_executable(bridge_node
    src/bridge_node.cpp
    src/bridge/bridge.cpp
)

if(BRIDGE_ENABLE_CAMERA)
  target_compile_definitions(bridge_node PRIVATE BRIDGE_ENABLE_CAMERA)
endif()

find_program(LD_GOLD_EXECUTABLE ld.gold)
if(LD_GOLD_EXECUTABLE)
  target_link_options(bridge_node PRIVATE -fuse-ld=gold)
endif()

target_include_directories(bridge_node PRIVATE
  include
  ${mavlink_INCLUDE_DIRS}
)

################################################################################
# Dependencies and linking
################################################################################
ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  visualization_msgs
  tf2_ros
  geometry_msgs
  dvl_msgs
  std_srvs
  bleurov_core_helper
  Eigen3
  sensor_msgs
  tf2_geometry_msgs
  rclcpp_lifecycle
  nav_msgs
  mavlink
  ament_index_cpp
)

if(BRIDGE_ENABLE_CAMERA)
  target_link_libraries(bridge_node PkgConfig::GSTREAMER)
endif()

if(BRIDGE_ENABLE_VIDEO_VIEWER)
  find_package(OpenCV REQUIRED COMPONENTS core highgui)
  add_executable(bluerov_video_viewer_node src/bluerov_video_viewer_node.cpp)
  target_include_directories(bluerov_video_viewer_node PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(bluerov_video_viewer_node ${OpenCV_LIBRARIES} PkgConfig::GSTREAMER)
  ament_target_dependencies(bluerov_video_viewer_node rclcpp sensor_msgs)

  install(TARGETS bluerov_video_viewer_node DESTINATION lib/${PROJECT_NAME})
  install(FILES launch/bluerov_video_viewer.launch.py DESTINATION share/${PROJECT_NAME}/launch)
  install(FILES param/bluerov_video.yaml DESTINATION share/${PROJECT_NAME}/param)
endif()

################################################################################
# Installation
################################################################################
install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/bag_recored
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
  OPTIONAL
)

install(DIRECTORY
  launch
  config
  models
  param
  docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "bluerov_video_viewer.launch.py" EXCLUDE
  PATTERN "bluerov_video.yaml" EXCLUDE
)

################################################################################
# Testing
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Finalize
################################################################################
ament_package()
