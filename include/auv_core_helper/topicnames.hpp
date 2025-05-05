#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace auv_core_helper {

namespace topicnames {

// PERCEPTION
const std::string obstacles = "/auv/perception/obstacles";

// CTRL
const std::string pose_desired = "/auv/pose_desired";
const std::string velocity_desired = "/auv/velocity_desired";
const std::string acceleration_desired = "/auv/acceleration_desired";
const std::string yaw_rate_desired = "/auv/yaw_rate_desired";
const std::string pose_goal= "/auv/pose_goal";

const std::string local_position_actual= "/auv/local_position";
const std::string global_position_actual = "/auv/global_position";
const std::string attitude_actual = "/auv/attitude";
const std::string dvl_distance_actual = "/auv/dvl_distance";

const std::string armed = "/auv/status/armed";
const std::string flight_mode = "/auv/status/flight_mode";

const std::string kcl_state = "/auv/kcl_state";
const std::string forces_desired = "/auv/forces_desired";
const std::string forces_desired_backseated = "/auv/forces_desired_backseated";

// SERVICES
const std::string control_cmd_service = "/auv/service/control_cmd";
}
}

#endif // TOPICNAMES_HPP
