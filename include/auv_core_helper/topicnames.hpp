#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace auv_core_helper {

namespace topicnames {

    // STATUS
const std::string system_status = "/auv/status";
const std::string ardusub_heartbeat = "/auv/heart_beat/ardusub";
const std::string bridge_heartbeat = "/auv/heart_beat/bridge";
const std::string perception_heartbeat = "/auv/image_pipeline/heart_beat";

const std::string safety_switch = "/auv/safety_switch";    
const std::string custom_switch = "/auv/custom_switch";    // The other switch

// MISSION
const std::string mission_status = "/auv/mission/status";

// PERCEPTION
const std::string objects = "/detections";

// CTRL
const std::string pose_desired_global = "/auv/global/pose_desired";
const std::string velocity_desired_global = "/auv/global/velocity_desired";


const std::string global_origin = "/auv/global/origin";
const std::string battery_status = "/auv/battery_status";
const std::string ekf_status = "/auv/ekf_status";

const std::string pose_actual_global_ = "/auv/global/pose_actual";
const std::string velocity_actual_global = "/auv/global/velocity_actual";
const std::string dvl_distance_actual = "/auv/dvl/distance";

const std::string kcl_state = "/auv/kcl_state";

const std::string desired_ctrl_mode = "/auv/desired_ctrl_mode";

const std::string gimbal_attitude_status = "/auv/gimbal/attitude_status";

// SERVICES
const std::string mission_cmd_service = "/auv/service/mission_cmd";
const std::string control_cmd_service = "/auv/service/control_cmd";
const std::string set_global_origin_service = "/auv/service/set_global_origin";
const std::string arming_service = "/auv/service/arming";
const std::string flight_mode_service = "/auv/service/flight_mode";
const std::string gimbal_service = "/auv/service/gimbal/attitude_desired";

// Actions
const std::string kcl_setter_action = "/set_kcl_state";

// Perceptions
const std::string obstacles = "/objects";
const std::string yolo_detections = "/objects";
}
}

#endif // TOPICNAMES_HPP    
