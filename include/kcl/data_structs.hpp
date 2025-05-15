#ifndef AUV_CONTROL_DATA_STRUCTS_HPP
#define AUV_CONTROL_DATA_STRUCTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>

namespace auv {

/// The `ControlData` structure encapsulates all control-related data for the AUV,
/// including state information, control goals, limits, and parameters for path planning,
/// velocity control, and state estimation.
struct ControlData {
    // ------------------------------
    // State Information
    // ------------------------------

    Eigen::VectorXd poseActualGlobal = Eigen::VectorXd(6); ///< Current AUV pose (lat, long, z, roll, pitch, yaw) in global coordinates.
    Eigen::VectorXd poseActualLocal = Eigen::VectorXd(6); ///<  Current AUV pose (x, y, z, roll, pitch, yaw).
    rclcpp::Time timeActual;               ///< Timestamp for the current pose.
    Eigen::VectorXd velocityActual = Eigen::VectorXd(6); ///< Current linear and angular velocities.
    Eigen::VectorXd accelerationActual = Eigen::VectorXd(6); ///< Current linear and angular accelerations.

    // ------------------------------
    // Desired State
    // ------------------------------
    // ------------------------------
    // LOCAL
    // ------------------------------
    Eigen::VectorXd poseGoalLocal = Eigen::VectorXd(6); ///< Desired pose goal.
    Eigen::VectorXd velocityDesiredLocal = Eigen::VectorXd(6); ///< Desired linear and angular velocities.
    

    // ------------------------------
    // GLOBAL
    // ------------------------------
    Eigen::VectorXd poseGoalGlobal = Eigen::VectorXd(6); ///< Desired pose goal in global coordinates.
    Eigen::VectorXd velocityDesiredGlobal = Eigen::VectorXd(6); ///< Desired linear and angular velocities in global coordinates.


    // ------------------------------
    // FLIGHT MODE PARAMETERS
    // ------------------------------
    bool armed_desired = false;
    bool armed_actual = false;
    std::string flightMode_desired = "MANUAL"; 
    std::string flightMode_actual = "MANUAL"; 


    // ------------------------------
    // Path Planning Parameters
    // ------------------------------
    int pathPlanningMode = 0; ///< Path planning mode: 2D (0) or 3D (1).

    // 2D Serpentine Path Parameters
    double serpentineAngle = 0.0; ///< Angle for 2D serpentine path planning.
    bool serpentineDirection = true; ///< Direction: true = forward, false = backward.
    double serpentineOffset = 0.0; ///< Offset for the serpentine path.
    std::vector<Eigen::Vector3d> serpentinePolygonVertices; ///< Polygon vertices for 2D serpentine planning.

    // 3D Serpentine Path Parameters
    double diveDepth = 0.0; ///< Dive depth for 3D serpentine path planning.
    double curvature = 0.0; ///< Path curvature for 3D serpentine paths.
    double dipNumPoints = 0.0; ///< Number of points in the serpentine "dip".
    double diveLength = 0.0; ///< Length of the dive.

    // 3D Helix Path Parameters
    Eigen::Vector3d helixStartPos = Eigen::Vector3d::Zero(); ///< Start position on the helix.
    Eigen::Vector3d helixAxisPos = Eigen::Vector3d::Zero(); ///< Point on the helix axis.
    Eigen::Vector3d helixAxisDir = Eigen::Vector3d::Zero(); ///< Direction of the helix axis.
    double helixFrequency = 0.0; ///< Length along the helix axis for one revolution.
    int helixNumQuadrants = 0; ///< Number of quadrants in the helix.
    bool helixCounterClockwise = true; ///< Revolution direction: true = counter-clockwise.

    // ------------------------------
    // Planned Path
    // ------------------------------
    nav_msgs::msg::Path plannedPath; ///< The planned path for the AUV.

    // ------------------------------
    // Control Gains
    // ------------------------------
    Eigen::VectorXd gainsX = Eigen::VectorXd(6); ///< Control gains for the X-axis.
    Eigen::VectorXd gainsY = Eigen::VectorXd(6); ///< Control gains for the Y-axis.
    Eigen::VectorXd gainsZ = Eigen::VectorXd(6); ///< Control gains for the Z-axis.
    Eigen::VectorXd gainsRoll = Eigen::VectorXd(6); ///< Control gains for roll.
    Eigen::VectorXd gainsPitch = Eigen::VectorXd(6); ///< Control gains for pitch.
    Eigen::VectorXd gainsYaw = Eigen::VectorXd(6); ///< Control gains for yaw.

    // ------------------------------
    // Velocity Limits
    // ------------------------------
    Eigen::VectorXd maxVelocity = Eigen::VectorXd(6); ///< Maximum allowed velocities (linear and angular).
    Eigen::VectorXd minVelocity = Eigen::VectorXd(6); ///< Minimum allowed velocities (linear and angular).

    double dt = 0.1; ///< Time step for control calculations.

    // ------------------------------
    // Constructor
    // ------------------------------
    ControlData() {
        // Initialize Eigen matrices and vectors with default values
        poseActualGlobal.setZero();
        velocityActual.setZero();
        accelerationActual.setZero();
        velocityDesiredLocal.setZero();
        poseGoalLocal.setZero();
        maxVelocity.setConstant(1.0); // Default maximum velocities.
        minVelocity.setConstant(0.0); // Default minimum velocities.
    }
};

} // namespace auv

#endif // AUV_CONTROL_DATA_STRUCTS_HPP
