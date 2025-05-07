#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace auv_core_helper {

namespace topicnames {

// PERCEPTION
const std::string obstacles = "/auv/perception/obstacles";

// CTRL
const std::string local_pose_desired = "/auv/local/pose_desired";
const std::string local_velocity_desired = "/auv/local/velocity_desired";
const std::string local_pose_goal= "/auv/local/pose_goal";
const std::string rc_channel_values_desired = "/auv/rc/channel_values_desired";

const std::string heart_beat = "/auv/heart_beat";
const std::string local_pose_actual= "/auv/local/pose";
const std::string local_velocity_actual = "/auv/local/velocity";
const std::string global_pose_actual = "/auv/global/pose";
const std::string global_velocity_actual = "/auv/global/velocity";
const std::string dvl_distance_actual = "/auv/dvl/distance";

const std::string kcl_state = "/auv/kcl_state";
const std::string forces_desired = "/auv/forces_desired";
const std::string forces_desired_backseated = "/auv/forces_desired_backseated";

// SERVICES
const std::string control_cmd_service = "/auv/service/control_cmd";
const std::string arming_service = "/auv/service/arming";
const std::string flight_mode_service = "/auv/service/flight_mode";
}
}

#endif // TOPICNAMES_HPP
