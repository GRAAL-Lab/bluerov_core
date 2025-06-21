#include "system_monitor/system_status_monitor.hpp"

using namespace mission;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemStatusMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}