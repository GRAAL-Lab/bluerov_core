cmake_minimum_required(VERSION 3.8)
project(bluerov_video_viewer)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0)

add_executable(bluerov_video_viewer_node src/bluerov_video_viewer_node.cpp)
target_compile_features(bluerov_video_viewer_node PUBLIC cxx_std_17)
target_include_directories(bluerov_video_viewer_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bluerov_video_viewer_node ${OpenCV_LIBRARIES} PkgConfig::GSTREAMER)
ament_target_dependencies(bluerov_video_viewer_node rclcpp sensor_msgs)

install(TARGETS bluerov_video_viewer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
