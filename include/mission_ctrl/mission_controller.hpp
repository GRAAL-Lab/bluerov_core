
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mission_ctrl/mission_data_structs.hpp"
#include "mission_ctrl/states/state_base.hpp"
#include "mission_ctrl/states/state_init.hpp"
#include "mission_ctrl/states/state_latlong.hpp"
#include "mission_ctrl/states/state_homing.hpp"
#include "mission_ctrl/states/state_search_object.hpp"
#include "mission_ctrl/states/state_cross_gate.hpp"
#include "mission_ctrl/states/state_search_buoy_area.hpp"
#include "mission_ctrl/states/state_inspect_buoy.hpp"
#include "mission_ctrl/states/state_inspect_pipes.hpp"


#include "auv_core_helper/topicnames.hpp"
#include "auv_core_helper/action/set_kcl.hpp"
#include "auv_core_helper/srv/mission_command.hpp"
#include "auv_core_helper/msg/mission_status.hpp"
#include "auv_core_helper/msg/dtc_list.hpp"

namespace mission {

class MissionController : public rclcpp::Node {

    std::shared_ptr<ControlData> ctrlData_;
    std::shared_ptr<TaskBenchmarkSettings> taskData_;

    fsm::FSM rFsm_;

    std::shared_ptr<states::StateInit> stateInit_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateSearchObject> stateSearchObject_;
    std::shared_ptr<states::StateCrossGate> stateCrossGate_;
    std::shared_ptr<states::StateHoming> stateHoming_;
    std::shared_ptr<states::StateSearchBuoyArea> stateSearchBuoyArea_;
    std::shared_ptr<states::StateInspectBuoy> stateInspectBuoy_;
    std::shared_ptr<states::StateInspectPipes> stateInspectPipes_;
    std::unordered_map<std::string, std::shared_ptr<states::StateBase>> statesMap_;

    rclcpp::Publisher<auv_core_helper::msg::MissionStatus>::SharedPtr missionStatusPub_;
    rclcpp_action::Client<auv_core_helper::action::SetKCL>::SharedPtr setKCLClient_;
    rclcpp::Service<auv_core_helper::srv::MissionCommand>::SharedPtr missionCommandService_;
    
    //auv_core_helper::msg::ObstacleList obstacles_;
    rclcpp::Subscription<auv_core_helper::msg::DtcList>::SharedPtr perceptionSub_;

    rclcpp::TimerBase::SharedPtr runTimer_; // Main function timer

    rclcpp::Time lastPerceptionTime_;
    //rclcpp::Time lastKCLTime_;

    bool LoadConfiguration();

    void SetUpFSM();
    void UpdateFSM();

    void Run();

    void StatusPub();

    void PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg);

    void MissionCommandCB(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request,
                           std::shared_ptr<auv_core_helper::srv::MissionCommand::Response> response);

public:
    MissionController();

};

}
// #endif // MISSION_CTRL_MISSION_CONTROLLER_HPP
