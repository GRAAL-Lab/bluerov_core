cmake_minimum_required(VERSION 3.8)
project(image_pipeline_obstacle_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(obstacle_tracking_msg REQUIRED)
find_package(detav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)

find_library(RML_LIBRARY NAMES rml)
find_library(FSM_LIBRARY NAMES fsm)
find_library(CTRL_LIBRARY NAMES ctrl_toolbox)
find_library(IKCL_LIBRARY NAMES ikcl)
find_library(LIBCONFIG_LIBRARY NAMES config++)
find_library(ODTC_LIBRARY NAMES odtc)
find_library(GEOGRAPHIC_LIBRARY NAMES Geographic)
link_directories(/usr/lib/x86_64-linux-gnu)
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "srv/*.srv"
    "*.xml"
    "launch/*.py"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${pcl_conversions_INCLUDE_DIRS}
  ${obstacle_tracking_msg_INCLUDE_DIRS}
  ${detav_msgs_INCLUDE_DIRS}
  ${ulisse_msgs_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(marine_detector_node src/marine_detector_node.cpp src/marine_detector.cpp src/marine_detector_ros2.cpp src/offline_detection_utils.cpp src/utilities_ros2.cpp)
ament_target_dependencies(marine_detector_node rclcpp message_filters std_msgs nav_msgs sensor_msgs PCL obstacle_tracking_msg detav_msgs geographic_msgs)
install(TARGETS
  marine_detector_node
  DESTINATION lib/${PROJECT_NAME})
target_include_directories(marine_detector_node PRIVATE include ${CMAKE_BINARY_DIR}/rosidl_generated_include ${detav_msgs_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${geographic_msgs_INCLUDE_DIRS})
target_link_libraries(marine_detector_node ${ament_index_cpp_LIBRARIES} ${ODTC_LIBRARY} ${RML_LIBRARY} ${GEOGRAPHIC_LIBRARY} ${pcl_conversions_LIBRARIES} ${PCL_LIBRARY} ${LIBCONFIG_LIBRARY} ${OpenCV_LIBS})

add_executable(marine_tracking_node src/marine_tracking_node.cpp src/marine_tracking_ros2.cpp src/offline_detection_utils.cpp src/utilities_ros2.cpp)
ament_target_dependencies(marine_tracking_node rclcpp message_filters std_msgs nav_msgs sensor_msgs PCL obstacle_tracking_msg detav_msgs geographic_msgs)
install(TARGETS
marine_tracking_node
  DESTINATION lib/${PROJECT_NAME})
target_include_directories(marine_tracking_node PRIVATE include ${CMAKE_BINARY_DIR}/rosidl_generated_include ${detav_msgs_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(marine_tracking_node ${ament_index_cpp_LIBRARIES} ${ODTC_LIBRARY} ${RML_LIBRARY} ${GEOGRAPHIC_LIBRARY} ${pcl_conversions_LIBRARIES} ${PCL_LIBRARY} ${LIBCONFIG_LIBRARY} ${OpenCV_LIBS})


add_executable(test_tmp src/test_pub.cpp)
ament_target_dependencies(test_tmp rclcpp message_filters std_msgs nav_msgs sensor_msgs PCL obstacle_tracking_msg detav_msgs geographic_msgs)
install(TARGETS
test_tmp
  DESTINATION lib/${PROJECT_NAME})
target_include_directories(test_tmp PRIVATE include ${CMAKE_BINARY_DIR}/rosidl_generated_include ${detav_msgs_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(test_tmp ${ament_index_cpp_LIBRARIES} ${ODTC_LIBRARY} ${RML_LIBRARY} ${GEOGRAPHIC_LIBRARY} ${pcl_conversions_LIBRARIES} ${PCL_LIBRARY} ${LIBCONFIG_LIBRARY} ${OpenCV_LIBS})



install(
  PROGRAMS
    scripts/python_yolo_publisher.py  # Path to your Python file (relative to the package root)
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
