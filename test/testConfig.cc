
#include "mission_ctrl/mission_controller.hpp"

using namespace rami;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionController>("example.conf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    

}