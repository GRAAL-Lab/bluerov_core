#ifndef MISSION_CTRL_HPP
#define MISSION_CTRL_HPP

#include "rclcpp/rclcpp.hpp"

#include "mission_ctrl/commands/generic_command.hpp"
#include "mission_ctrl/mission_data_structs.hpp"
#include "mission_ctrl/states/generic_state.hpp"

namespace rami {

class MissionController : public rclcpp::Node {

    std::string fileName_;
    std::shared_ptr<TaskBenchMarkSettings> conf_;

    bool LoadConfiguration(std::shared_ptr<TaskBenchMarkSettings>& conf);

public:
    MissionController(std::string conf_filename);
};
}
#endif // MISSION_CTRL_HPP
