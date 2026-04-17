cmake_minimum_required(VERSION 3.8)
project(bridge)

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
find_package(auv_core_helper REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(mavlink REQUIRED)
find_package(std_srvs REQUIRED)

pkg_check_modules(LIBCONFIGPP REQUIRED libconfig++)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# Prefer shared libraries over static ones
set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${auv_core_helper_INCLUDE_DIRS}
  ${LIBCONFIGPP_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
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

find_program(LD_GOLD_EXECUTABLE ld.gold)
if(LD_GOLD_EXECUTABLE)
  target_link_options(bridge_node PRIVATE -fuse-ld=gold)
endif()

target_include_directories(bridge_node PRIVATE
  include
  ${auv_core_helper_INCLUDE_DIRS}
  ${LIBCONFIGPP_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

################################################################################
# Dependencies and linking
################################################################################
ament_target_dependencies(bridge_node
  rclcpp
  visualization_msgs
  tf2_ros
  geometry_msgs
  Eigen3
  sensor_msgs
  tf2_geometry_msgs
  auv_core_helper
  rclcpp_lifecycle
  nav_msgs
  mavlink
  std_srvs
  ament_index_cpp
)

target_link_libraries(bridge_node
  ${auv_core_helper_LIBRARIES}
  ${LIBCONFIGPP_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

################################################################################
# Installation
################################################################################
install(TARGETS bridge_node
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
