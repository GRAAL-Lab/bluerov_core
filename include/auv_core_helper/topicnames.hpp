#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace auv_core_helper {

namespace topicnames {

// MISSION
const std::string mission_status = "/auv/mission/status";

// PERCEPTION
const std::string objects = "/auv/perception/objects";

// CTRL
const std::string pose_desired_local = "/auv/local/pose_desired";
const std::string velocity_desired_local = "/auv/local/velocity_desired";

const std::string pose_desired_global = "/auv/global/pose_desired";
const std::string velocity_desired_global = "/auv/global/velocity_desired";
const std::string pose_goal_local= "/auv/local/pose_goal";


const std::string pose_actual_local= "/auv/local/pose_actual";
const std::string velocity_actual_local = "/auv/local/velocity_actual";
const std::string acceleration_actual_local = "/auv/local/acceleration_actual";


const std::string rc_channel_values_desired = "/auv/rc/channel_values_desired";
const std::string heart_beat = "/auv/heart_beat";
const std::string battery_status = "/auv/battery_status";

const std::string pose_actual_global_ = "/auv/global/pose_actual";
const std::string velocity_actual_global = "/auv/global/velocity_actual";
const std::string dvl_distance_actual = "/auv/dvl/distance";

const std::string kcl_state = "/auv/kcl_state";

// SERVICES
const std::string control_cmd_service = "/auv/service/control_cmd";
const std::string arming_service = "/auv/service/arming";
const std::string flight_mode_service = "/auv/service/flight_mode";
}
}

#endif // TOPICNAMES_HPP    
