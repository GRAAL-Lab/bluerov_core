#ifndef HELPER_LIB_HPP
#define HELPER_LIB_HPP

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "bleurov_core_helper/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <libconfig.h++>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> 

//using namespace Eigen;

// Declaration of functions
void LoadParamsFromConf(const std::string& config_name, Eigen::VectorXd* thruster_upper_limits,
                        Eigen::VectorXd* thruster_lower_limits, Eigen::VectorXd* thruster_allocation_weights,
                        Eigen::VectorXd* gainsX, Eigen::VectorXd* gainsY, Eigen::VectorXd* gainsZ,
                        Eigen::VectorXd* gainsRoll, Eigen::VectorXd* gainsPitch, Eigen::VectorXd* gainsYaw,
                        Eigen::VectorXd* max_linear_angular_velocities, Eigen::VectorXd* min_linear_angular_velocities);

void PublishEigenPose(const rclcpp::Publisher<bleurov_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time);
void PublishEigenVelocity(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& velocity);
void PublishEigenAcceleration(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& acceleration);
Eigen::Vector3d ConvertAngularVelocitiesToEulerRates(double rollActual, double pitchActual, const Eigen::Vector3d& bOmegaDesired);

namespace bleurov_core_helper {

struct VelocityRcOverrideConfig {
    bool enabled{true};
    bool require_poshold{true};
    std::string frame_id{"auv_base_link"};
    double rate_hz{100.0};
    double command_timeout_s{0.25};
    std::array<double, 6> velocity_min_flu{{-2.0, -1.8, -0.55, -2.0, -2.2, -1.85}};
    std::array<double, 6> velocity_max_flu{{2.0, 1.8, 0.55, 2.0, 2.2, 1.85}};
    std::array<uint8_t, 6> rc_channels{{1, 2, 3, 4, 6, 7}};
    std::array<uint16_t, 4> pwm_min{{1100, 1100, 1100, 1100}};
    std::array<uint16_t, 4> pwm_neutral{{1500, 1500, 1500, 1500}};
    std::array<uint16_t, 4> pwm_max{{1900, 1900, 1900, 1900}};
    uint16_t attitude_neutral_pwm{1500};
};

bool LoadVelocityRcOverrideConfig(VelocityRcOverrideConfig* config);

}  // namespace bleurov_core_helper

void LoadBridgeParamsFromConf(const std::string& config_name, bool* simulation_mode, std::string* remote_addr, int* system_id, int* component_id, int* port,
                              bool* camera_enabled = nullptr, std::string* camera_source_uri = nullptr, std::string* camera_topic = nullptr,
                              std::string* camera_info_topic = nullptr, std::string* camera_frame_id = nullptr, bool* camera_use_hw_decoder = nullptr,
                              bool* camera_qos_reliable = nullptr, int* camera_preview_width = nullptr, int* camera_preview_height = nullptr,
                              double* camera_preview_max_fps = nullptr, int* camera_rtp_latency_ms = nullptr,
                              std::string* camera_rtp_caps = nullptr, bool* camera_enable_max_performance = nullptr);

void LoadBridgeParamsFromConf(const std::string& config_name, bool* simulation_mode, std::string* remote_addr, int* system_id, int* component_id, int* port,
                              bool* camera_enabled = nullptr, std::string* camera_source_uri = nullptr, std::string* camera_topic = nullptr,
                              std::string* camera_info_topic = nullptr, std::string* camera_frame_id = nullptr, bool* camera_use_hw_decoder = nullptr,
                              bool* camera_qos_reliable = nullptr, int* camera_preview_width = nullptr, int* camera_preview_height = nullptr,
                              double* camera_preview_max_fps = nullptr, std::string* camera_output_encoding = nullptr,
                              double* camera_info_publish_rate_hz = nullptr, int* camera_rtp_latency_ms = nullptr,
                              std::string* camera_rtp_caps = nullptr, bool* camera_enable_max_performance = nullptr);

#endif // HELPER_LIB_HPP
