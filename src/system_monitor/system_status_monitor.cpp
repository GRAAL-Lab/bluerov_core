#include "system_monitor/system_status_monitor.hpp"

namespace mission {
SystemStatusMonitor::SystemStatusMonitor()
    : Node("system_status_monitor_node")
{
    //     LoadConfiguration(); // REQUIRES SYSTEM STATUS TO BE INITIALIZED

    lastSystemTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    lastMissionCtrlTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    lastBridgeTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    lastKCLTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    lastPerceptionTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    missionCtrlSub_ = this->create_subscription<auv_core_helper::msg::MissionStatus>(
        auv_core_helper::topicnames::mission_status, rclcpp::SystemDefaultsQoS(),
        std::bind(&SystemStatusMonitor::MissionCtrlCB, this, std::placeholders::_1));

    bridgeSub_ = this->create_subscription<auv_core_helper::msg::HeartBeat>(
        auv_core_helper::topicnames::heart_beat, rclcpp::SystemDefaultsQoS(),
        std::bind(&SystemStatusMonitor::BridgeCB, this, std::placeholders::_1));

    kclSub_ = this->create_subscription<auv_core_helper::msg::KclStatus>(
        auv_core_helper::topicnames::kcl_state, rclcpp::SystemDefaultsQoS(),
        std::bind(&SystemStatusMonitor::KclCB, this, std::placeholders::_1));
    serverKclClient_ = rclcpp_action::create_client<auv_core_helper::action::SetKCL>(
        this, auv_core_helper::topicnames::kcl_setter_action);

    perceptionSub_ = this->create_subscription<auv_core_helper::msg::DtcList>(
        auv_core_helper::topicnames::objects, rclcpp::SystemDefaultsQoS(),
        std::bind(&SystemStatusMonitor::PerceptionCB, this, std::placeholders::_1));

    int pub_rate = 1; // Default rateù
    std::chrono::milliseconds pub_duration(1000 / pub_rate);
    runTimer_ = this->create_wall_timer(pub_duration, std::bind(&SystemStatusMonitor::StatusPub, this));
    systemStatusPub_ = this->create_publisher<auv_core_helper::msg::SystemStatus>(
        auv_core_helper::topicnames::system_status, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(this->get_logger(), "System Status Monitor is running.");
};

void SystemStatusMonitor::StatusPub()
{
    auv_core_helper::msg::SystemStatus status;
    status.stamp = this->get_clock()->now();
    status.mission_ctrl = true;
    status.kcl = true;
    status.perception = true;
    status.bridge = true;
    status.system_operational = true;

    auto timeSinceLastMissionCtrl = this->get_clock()->now() - lastMissionCtrlTime;
    auto timeSinceLastBridge = this->get_clock()->now() - lastBridgeTime;
    auto timeSinceLastKCL = this->get_clock()->now() - lastKCLTime;
    auto timeSinceLastPerception = this->get_clock()->now() - lastPerceptionTime;

    if (timeSinceLastMissionCtrl.seconds() > missionCtrlTimeout_) {
        status.mission_ctrl = false;
    }

    if (timeSinceLastBridge.seconds() > bridgeTimeout_) {
        status.bridge = false;
        status.system_operational = false;
    }

    if (timeSinceLastKCL.seconds() > kclTimeout_) {
        status.kcl = false;
        status.system_operational = false;
    }
    if (!serverKclClient_->wait_for_action_server(std::chrono::seconds(1))) {
        status.kcl = false;
        status.system_operational = false;
    }

    if (timeSinceLastPerception.seconds() > perceptionTimeout_) {
        status.perception = false;
        status.system_operational = false;
    } 

    if(status.system_operational) {
        lastSystemTime = this->get_clock()->now();
    }

    systemStatusPub_->publish(status);
    RCLCPP_DEBUG(this->get_logger(), "System Status: %s \n  - Mission Ctrl: %s, \n  - KCL: %s, \n  - Perception: %s, \n  - Bridge: %s",
        status.mission_ctrl ? "Alive" : "Dead",
        status.kcl ? "Alive" : "Dead",
        status.perception ? "Alive" : "Dead",
        status.bridge ? "Alive" : "Dead");
}

void SystemStatusMonitor::MissionCtrlCB(const auv_core_helper::msg::MissionStatus::SharedPtr msg)
{
    (void)msg; // Unused parameter
    lastMissionCtrlTime = this->get_clock()->now();
}

void SystemStatusMonitor::BridgeCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg)
{
    (void)msg; // Unused parameter
    lastBridgeTime = this->get_clock()->now();
}

void SystemStatusMonitor::KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg)
{
    (void)msg;
    lastKCLTime = this->get_clock()->now();
}

void SystemStatusMonitor::PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg)
{   
    (void)msg; // Unused parameter
    lastPerceptionTime = this->get_clock()->now();
}

void SystemStatusMonitor::LoadConfiguration()
{
    
    // Load configuration from file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("auv_core_helper");
    std::string confPath = package_share_directory + "/param/" + "system_monitor.conf";
    libconfig::Config confObj;

    try {
        confObj.readFile(confPath.c_str());
        ctb::GetParam(confObj, runRate_, "check_rate");
        ctb::GetParam(confObj, missionCtrlTimeout_, "mission_ctrl_timeout");
        ctb::GetParam(confObj, bridgeTimeout_, "bridge_timeout");
        ctb::GetParam(confObj, kclTimeout_, "kcl_timeout");
        ctb::GetParam(confObj, perceptionTimeout_, "perception_timeout");

    } catch (const libconfig::FileIOException& fioex) {
        RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
        RCLCPP_ERROR(this->get_logger(), "  Path: '%s'. Make sure the file exists and is readable.", confPath.c_str());
    } catch (const libconfig::ParseException& pex) {
        RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d - %s", pex.getFile(), pex.getLine(), pex.getError());
    }
}

} // namespace mission