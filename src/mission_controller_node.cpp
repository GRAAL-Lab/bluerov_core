#include "mission_ctrl/mission_controller.hpp"

using namespace mission;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionController>("tasks.conf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}