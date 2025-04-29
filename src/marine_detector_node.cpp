#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <marine_detector_ros2.h>

int main(int argc, char * argv[]) {
  std::cerr << tc::bluL << "[MarineDetectorNode] Starting..." << tc::none << std::endl;
  rclcpp::init(argc, argv);
  if (argc < 3) {
    throw std::runtime_error("Not enough inputs! <settings>");
  }
  if (std::string(argv[1]) != "--settings") {
    throw std::runtime_error("Wrong inputs! <settings>");
  }
  std::string settingsPath(argv[2]);
  rclcpp::spin(std::make_shared<MarineDetectorROS2>(settingsPath,true));
  rclcpp::shutdown();
  std::cerr << tc::greenL << "[MarineDetectorNode] End!" << tc::none << std::endl;
  return 0;
}