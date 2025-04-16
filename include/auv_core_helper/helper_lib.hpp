#ifndef HELPER_LIB_HPP
#define HELPER_LIB_HPP

#include <Eigen/Dense>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "auv_core_helper/msg/pose_stamped.hpp"
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

void PublishEigenPose(const rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time);
void PublishEigenVelocity(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& velocity);
void PublishEigenAcceleration(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& acceleration);
Eigen::Vector3d ConvertAngularVelocitiesToEulerRates(double rollActual, double pitchActual, const Eigen::Vector3d& bOmegaDesired);
#endif // HELPER_LIB_HPP
