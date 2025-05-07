cmake_minimum_required(VERSION 3.5)
project(test_package)

include_directories("${CMAKE_SOURCE_DIR}/include/jsoncons/include")
include_directories("${CMAKE_SOURCE_DIR}/include/ctrl_toolbox/include/")
include_directories("${CMAKE_SOURCE_DIR}/include/json_utils")

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
# Link against the json_utils library
target_link_libraries(my_test
  json_utils
)

ament_target_dependencies(my_test rclcpp std_msgs)

install(TARGETS my_test DESTINATION lib/${PROJECT_NAME})

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
