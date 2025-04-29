
#include "rclcpp/rclcpp.hpp"


#include "mission_ctrl/mission_data_structs.hpp"
#include "mission_ctrl/states/state_base.hpp"
#include "mission_ctrl/states/state_init.hpp"
#include "mission_ctrl/states/state_latlong.hpp"
#include "mission_ctrl/states/state_homing.hpp"

#include "auv_core_helper/msg/obstacle_list.hpp"
#include "auv_core_helper/topicnames.hpp"

// #include "auv_core_helper/msg/pose_stamped.hpp"

namespace mission {

class MissionController : public rclcpp::Node {

    std::string fileName_;

    std::shared_ptr<ControlData> ctrlData_;
    std::shared_ptr<TaskBenchmarkSettings> taskData_;

    fsm::FSM rFsm_;

    std::shared_ptr<states::StateInit> stateInit_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateHoming> stateHoming_;
    std::unordered_map<std::string, std::shared_ptr<states::StateBase>> statesMap_;

    auv_core_helper::msg::ObstacleList obstacles_;
    rclcpp::Subscription<auv_core_helper::msg::ObstacleList>::SharedPtr obstaclesSub_;

    rclcpp::TimerBase::SharedPtr runTimer_; // Main function timer

    bool LoadConfiguration(std::shared_ptr<TaskBenchmarkSettings>& conf);

    void SetUpFSM();

public:
    MissionController(std::string conf_filename);

    void Run();
};

}
// #endif // MISSION_CTRL_MISSION_CONTROLLER_HPP
