#include "ctrl_toolbox/HelperFunctions.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "auv_core_helper/action/set_kcl.hpp"
#include "auv_core_helper/msg/dtc_list.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"
#include "auv_core_helper/msg/kcl_status.hpp"
#include "auv_core_helper/msg/mission_status.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/msg/system_status.hpp"
#include "auv_core_helper/topicnames.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mission {

class SystemStatusMonitor : public rclcpp::Node {

    int runRate_ = 1; // Hz
    double missionCtrlTimeout_ = 2.0;
    double bridgeTimeout_ = 2.0;
    double kclTimeout_ = 2.0;
    double perceptionTimeout_ = 2.0;
    double vehicleMovingToSafetyAreaTimeout_ = 60.0;

    bool vehicleIsFreeToMove_ = true; // false if it gets stuck (not implemented yet)
    bool safetySwitchIsOff_ = true; // At start of mission
    bool vehicleIsInSafetyArea_ = true;
    bool vehicleReachedSafetyArea_ = false; // At start of mission
    std::vector<ctb::LatLong> safetyBoundary_;

    // Pubs and Subs, action client to KCL and service for mission command
    rclcpp::Publisher<auv_core_helper::msg::SystemStatus>::SharedPtr systemStatusPub_;
    rclcpp::Subscription<auv_core_helper::msg::MissionStatus>::SharedPtr missionCtrlSub_;
    rclcpp::Subscription<auv_core_helper::msg::HeartBeat>::SharedPtr bridgeHeartBeathSub_;
    rclcpp::Subscription<auv_core_helper::msg::HeartBeat>::SharedPtr ardusubHeartBeathSub_;
    rclcpp::Subscription<auv_core_helper::msg::KclStatus>::SharedPtr kclSub_;
    rclcpp_action::Client<auv_core_helper::action::SetKCL>::SharedPtr serverKclClient_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr perceptionSub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySwitchSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr customSwitchSub_;

    bool rcvFirstPose_ = false;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseSub_;

    rclcpp::Time lastMissionCtrlTime;
    rclcpp::Time lastBridgeTime;
    rclcpp::Time lastArdusubTime;
    rclcpp::Time lastKCLTime;
    rclcpp::Time lastPerceptionTime;

    bool missionUnderExecution = false;
    rclcpp::Time rcvMissionCmdTime;
    std::string missionCtrlStatus_ = "Unknown";

    rclcpp::TimerBase::SharedPtr runTimer_;

    // Callbacks
    void StatusPub();
    void MissionCtrlCB(const auv_core_helper::msg::MissionStatus::SharedPtr msg);
    void BridgeCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg);
    void ArdusubCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg);
    void KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg);
    void PerceptionCB(const std_msgs::msg::Bool::SharedPtr msg);

    void SafetySwitchCB(const std_msgs::msg::Bool::SharedPtr msg);
    void CustomSwitchCB(const std_msgs::msg::Bool::SharedPtr msg);

    void PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg);

    void LoadConfiguration();

    bool IsPointWithinBoundaries(const ctb::LatLong& point);

public:
    SystemStatusMonitor();
};

}
// #endif // MISSION_CTRL_MISSION_CONTROLLER_HPP
