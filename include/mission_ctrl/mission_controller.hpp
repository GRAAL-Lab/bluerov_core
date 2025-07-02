
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cmath>

#include "mission_ctrl/mission_data_structs.hpp"
#include "mission_ctrl/states/state_base.hpp"
#include "mission_ctrl/states/state_cross_gate.hpp"
#include "mission_ctrl/states/state_homing.hpp"
#include "mission_ctrl/states/state_init.hpp"
#include "mission_ctrl/states/state_inspect_buoy.hpp"
#include "mission_ctrl/states/state_inspect_pipes.hpp"
#include "mission_ctrl/states/state_latlong.hpp"
#include "mission_ctrl/states/state_depth.hpp"
#include "mission_ctrl/states/state_search_buoy_area.hpp"
#include "mission_ctrl/states/state_search_object.hpp"
#include "mission_ctrl/states/state_update_localization.hpp"

#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/action/set_kcl.hpp"
#include "auv_core_helper/msg/dtc_list.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"
#include "auv_core_helper/msg/kcl_status.hpp"
#include "auv_core_helper/msg/mission_status.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/msg/system_status.hpp"
#include "auv_core_helper/srv/mission_command.hpp"
#include "auv_core_helper/srv/set_gimbal_attitude.hpp"
#include "auv_core_helper/topicnames.hpp"

namespace mission {

class MissionController : public rclcpp::Node {

    std::shared_ptr<SystemStatus> systemStatus_;
    std::shared_ptr<ControlData> ctrlData_;
    std::shared_ptr<TaskBenchmarkSettings> taskData_;

    // FSM
    fsm::FSM rFsm_;
    std::unordered_map<std::string, std::shared_ptr<states::StateBase>> statesMap_;
    std::shared_ptr<states::StateInit> stateInit_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateDepth> stateDepth_;
    std::shared_ptr<states::StateSearchObject> stateSearchObject_;
    std::shared_ptr<states::StateCrossGate> stateCrossGate_;
    std::shared_ptr<states::StateHoming> stateHoming_;
    std::shared_ptr<states::StateSearchBuoyArea> stateSearchBuoyArea_;
    std::shared_ptr<states::StateInspectBuoy> stateInspectBuoy_;
    std::shared_ptr<states::StateInspectPipes> stateInspectPipes_;
    std::shared_ptr<states::StateUpdateLocalization> stateUpdateLocalization_;
    std::shared_ptr<states::StateSleep> stateSleep_;

    // Pubs and Subs, action client to KCL and service for mission command
    rclcpp::Publisher<auv_core_helper::msg::MissionStatus>::SharedPtr missionStatusPub_;
    rclcpp::Subscription<auv_core_helper::msg::SystemStatus>::SharedPtr systemStatusSub_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseSub_;
    rclcpp::Subscription<auv_core_helper::msg::DtcList>::SharedPtr perceptionSub_;
    rclcpp::Subscription<auv_core_helper::msg::KclStatus>::SharedPtr KclSub_;
    rclcpp_action::Client<auv_core_helper::action::SetKCL>::SharedPtr setKCLClient_;
    rclcpp::Service<auv_core_helper::srv::MissionCommand>::SharedPtr missionCommandService_;
    rclcpp::Client<auv_core_helper::srv::SetGimbalAttitude>::SharedPtr setGimbalAttitudeService_;
    double lastSetGimbalAttitude_ = 0.0;

    std_msgs::msg::String debugMsg;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debugPub_;

    rclcpp::TimerBase::SharedPtr runTimer_;
    rclcpp::TimerBase::SharedPtr delayMissionStartTimer_;
    rclcpp::TimerBase::SharedPtr simCtrlStationTimer_;

    rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions kclSendGoalOptions_;

    // bool latestMissionCmdSet_ = false;
    // bool restartingLatestMissionCmd_ = false;
    // auv_core_helper::srv::MissionCommand::Request::SharedPtr latestMissionCmdRequest_;

    void SimulateMissionCmdFromFile();
    void LoadConfiguration();

    // bool IsPointWithinBoundaries(const ctb::LatLong& point);
    bool kclCmd();
    bool kclStopCmd();
    bool kclCancelCmd();
    void SetGimbalAttitude();

    // FSM
    void SetUpFSM();
    void SetTaskDataFSM();
    void ResetTaskDataFSM();
    void Run();
    
    bool StartMission();

    // Pubs
    void StatusPub();

    // Callbacks
    void SystemStatusCB(const auv_core_helper::msg::SystemStatus::SharedPtr msg);
    void PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg);
    void KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg);
    void MissionCommandCB(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request,
        std::shared_ptr<auv_core_helper::srv::MissionCommand::Response> response);
    void ActionResultCallback(const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result);
    void ActionFeedbackCallback(
        rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::SharedPtr,
        const std::shared_ptr<const auv_core_helper::action::SetKCL::Feedback>& feedback);

public:
    MissionController();
};

}
// #endif // MISSION_CTRL_MISSION_CONTROLLER_HPP
