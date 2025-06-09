cmake_minimum_required(VERSION 3.8)
project(auv_core_helper)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(action_msgs REQUIRED)

# Collect all .msg and .srv files
file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB_RECURSE SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

# Define the helper library
add_library(${PROJECT_NAME}_lib SHARED
  src/helper_lib.cpp
)

# Ensure that the helper library depends on the generated messages/services
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})

# Add dependencies to the helper library
ament_target_dependencies(${PROJECT_NAME}_lib
  Eigen3
  rclcpp
  geometry_msgs
  PkgConfig
  rosidl_default_runtime
  action_msgs
)

# Include directories for the helper library
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${PkgConfig_INCLUDE_DIRS}
)

# Link additional libraries
target_link_libraries(${PROJECT_NAME}_lib
  config++
)

# Export dependencies and include directories
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  PkgConfig
  rclcpp
  Eigen3
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

# Install include files (modified to include all subdirectories)
install(DIRECTORY include/
  DESTINATION include/
)

# Install the generated message and service headers
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install library
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install param directory if present
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
