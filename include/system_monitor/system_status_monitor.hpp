#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>
#include "ctrl_toolbox/HelperFunctions.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "auv_core_helper/msg/system_status.hpp"
#include "auv_core_helper/msg/mission_status.hpp"
#include "auv_core_helper/action/set_kcl.hpp"
#include "auv_core_helper/msg/dtc_list.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"
#include "auv_core_helper/msg/kcl_status.hpp"
#include "auv_core_helper/msg/mission_status.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/srv/mission_command.hpp"
#include "auv_core_helper/topicnames.hpp"

namespace mission {

class SystemStatusMonitor : public rclcpp::Node {

    //bool systemInit = false;
    int runRate_ = 1; // Hz
    double missionCtrlTimeout_ = 2.0;
    double bridgeTimeout_ = 2.0;
    double kclTimeout_ = 2.0;
    double perceptionTimeout_ = 2.0;

    // Pubs and Subs, action client to KCL and service for mission command
    rclcpp::Publisher<auv_core_helper::msg::SystemStatus>::SharedPtr systemStatusPub_;
    rclcpp::Subscription<auv_core_helper::msg::MissionStatus>::SharedPtr missionCtrlSub_;
    rclcpp::Subscription<auv_core_helper::msg::HeartBeat>::SharedPtr bridgeSub_;
    rclcpp::Subscription<auv_core_helper::msg::KclStatus>::SharedPtr kclSub_;
    rclcpp_action::Client<auv_core_helper::action::SetKCL>::SharedPtr serverKclClient_;
    rclcpp::Subscription<auv_core_helper::msg::DtcList>::SharedPtr perceptionSub_;

    bool rcvFirstPose_ = false;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseSub_;

    rclcpp::Time lastSystemTime;
    rclcpp::Time lastMissionCtrlTime;
    rclcpp::Time lastBridgeTime;
    rclcpp::Time lastKCLTime;
    rclcpp::Time lastPerceptionTime;


    rclcpp::TimerBase::SharedPtr runTimer_;


    // Callbacks
    void StatusPub();
    void MissionCtrlCB(const auv_core_helper::msg::MissionStatus::SharedPtr msg);
    void BridgeCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg);
    void KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg);
    void PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg);

    void PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg);

    void LoadConfiguration();


public:
    SystemStatusMonitor();
};

}
// #endif // MISSION_CTRL_MISSION_CONTROLLER_HPP
