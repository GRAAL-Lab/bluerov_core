cmake_minimum_required(VERSION 3.5)
project(mission_ctrl)

#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rml REQUIRED)
find_package(fsm REQUIRED)
find_package(ctrl_toolbox REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(sisl_toolbox REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} sisl)

include_directories(
  include
)

set(use_SYSTEM_BOOST TRUE)

## Main Libraries and depedencies ##
set(mission_ctrl_lib_sources
    src/mission_ctrl/mission_controller.cpp

)

## Libraries
add_library(mission_ctrl_lib ${mission_ctrl_lib_sources})
ament_target_dependencies(mission_ctrl_lib rclcpp std_msgs ament_index_cpp)
target_link_libraries(mission_ctrl_lib ${catkin_LIBRARIES} config++ Geographic rml fsm ctrl_toolbox sisl_toolbox)

## Executables
add_executable(config_test test/testConfig.cc)
ament_target_dependencies(config_test rclcpp ament_index_cpp)
target_link_libraries(config_test rml mission_ctrl_lib)
# add_executable(control_test test/control_test.cpp)
# ament_target_dependencies(control_test rclcpp std_msgs ulisse_msgs ament_index_cpp  ctrl_toolbox rml)
# target_link_libraries(control_test rml mission_ctrl_lib ${ulisse_driver_LIBRARIES})

ament_export_include_directories(include)

## Install
install(TARGETS
  config_test
    DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY
  #launch
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
