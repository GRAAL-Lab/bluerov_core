#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace auv_core_helper {

namespace topicnames {

// CTRL
const std::string pose_desired = "/auv/pose_desired";
const std::string velocity_desired = "/auv/velocity_desired";
const std::string acceleration_desired = "/auv/acceleration_desired";
const std::string pose_goal= "/auv/pose_goal";
const std::string pose_actual= "/auv/pose_actual";
const std::string velocity_actual = "/auv/velocity_actual";
const std::string acceleration_actual = "/auv/acceleration_actual";
const std::string kcl_state = "/auv/kcl_state";
const std::string forces_desired = "/auv/forces_desired";
const std::string forces_desired_backseated = "/auv/forces_desired_backseated";

// SERVICES
const std::string control_cmd_service = "/auv/service/control_cmd";
}
}

#endif // TOPICNAMES_HPP
