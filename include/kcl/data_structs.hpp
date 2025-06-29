#ifndef AUV_CONTROL_DATA_STRUCTS_HPP
#define AUV_CONTROL_DATA_STRUCTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>
#include <ctrl_toolbox/DataStructs.h>
#include "auv_core_helper/bridgemode.hpp"


namespace auv {

/// The `ControlData` structure encapsulates all control-related data for the AUV,
/// including state information, control goals, limits, and parameters for path planning,
/// velocity control, and state estimation.
struct ControlData {
    // ------------------------------
    // State Information
    // ------------------------------

    rclcpp::Time timeActual;               ///< Timestamp for the current pose.
    Eigen::VectorXd velocityActual = Eigen::VectorXd(6); ///< Current linear and angular velocities.


    // ------------------------------
    // LOCAL
    // ------------------------------
    Eigen::VectorXd poseGoalLocal = Eigen::VectorXd(6); ///< Desired pose goal.
    Eigen::VectorXd poseActualLocal = Eigen::VectorXd(6); ///<  Current AUV pose (x, y, z, roll, pitch, yaw).
    Eigen::VectorXd homeLocal = Eigen::VectorXd(6); ///< Home position in local coordinates (x, y, z, roll, pitch, yaw).
    Eigen::VectorXd velocityDesiredNED = Eigen::VectorXd(6); ///< Desired linear and angular velocities.

    Eigen::VectorXd circularCenterLocal = Eigen::VectorXd(6); /// Center point for the circular path in local coordinates (x, y, z, roll, pitch, yaw).
    

    // ------------------------------
    // GLOBAL
    // ------------------------------
    Eigen::VectorXd poseActualGlobal = Eigen::VectorXd(6); ///< Current AUV pose (lat, long, z, roll, pitch, yaw) in global coordinates.
    Eigen::VectorXd homeGlobal = Eigen::VectorXd(6);
    ctb::LatLong homeLL; ///< Home position in global coordinates (latitude, longitude).
    ctb::LatLong poseActualLL; ///< Current AUV pose in global coordinates (latitude, longitude).
    Eigen::VectorXd poseGoalGlobal = Eigen::VectorXd(6); ///< Desired pose goal in global coordinates.
    Eigen::VectorXd velocityDesiredGlobal = Eigen::VectorXd(6); ///< Desired linear and angular velocities in global coordinates.


    // ------------------------------
    // FLIGHT MODE PARAMETERS
    // ------------------------------
    bool armed_desired = false;
    bool armed_actual = false;
    std::string flightMode_desired = "MANUAL"; 
    std::string flightMode_actual = "MANUAL"; 
    std::string deisiredCtrlMode = "NOT_SET"; ///< Desired control mode as string.

    // ------------------------------
    // Action State
    // ------------------------------
    bool actionSuccess = false; ///< Indicates if the last action was successful.
    bool actionFailed = false; ///< Indicates if the last action failed.
    std::string actionMessage = ""; ///< Message describing the result of the last action.
    std::string actualState = "IDLE"; ///< Current state of the AUV control system.
    double actionProgress = 0.0; ///< Progress of the current action, from 0.0 to 100.0 %.

    // ------------------------------
    // Path Planning Parameters
    // ------------------------------
    std::string pathPlanningMode = "Serpentine2D"; ///< Path planning mode as string.

    // 2D Serpentine Path Parameters
    //TO DO: MOVE TO PARAM FILE 
    Eigen::MatrixXd pathArea; /// Area for 2D serpentine path planning, defined by rows (latitude, longitude) and columns (vertices).
    double serpentineAngle = 90.0; ///< Angle for 2D serpentine path planning.
    bool serpentineDirection = true; ///< Direction: true = forward, false = backward.
    double serpentineOffset = 1.0; ///< Offset for the serpentine path.
    std::vector<Eigen::Vector3d> serpentinePolygonVertices; ///< Polygon vertices for 2D serpentine planning.

    // 2D Spiral Path Parameters
    double spiralDiameter = 0.0; ///< Diameter for 2D spiral path planning.
    double spiralIncrement = 0.0; ///< Increment for the spiral path.
    bool resumePath = false; ///< Flag to indicate if the path should be resumed.

    // 2D Circular Path Parameters
    double circularDiameter = 0.0; ///< Diameter for 2D circular path planning
    ctb::LatLong circularCenterLL; ///< Center point for the circular path in LatLong format.
    ctb::LatLong circularStartPointLL; ///< Starting point for the circular path in LatLong format.
    bool circularClockwise = true; ///< Direction of the circular path: true = clockwise,
    

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
        poseActualLocal.setZero();
        velocityActual.setZero();
        velocityDesiredNED.setZero();
        poseGoalLocal.setZero();
        poseGoalGlobal.setZero();
        velocityDesiredGlobal.setZero();
        homeLocal.setZero();
        homeGlobal.setZero();
        homeGlobal(0) = 44.096058; // Default home latitude.
        homeGlobal(1) = 9.864761;  // Default home longitude.
        homeGlobal(2) = 0.0;       // Default home depth.
        maxVelocity.setConstant(1.0); // Default maximum velocities.
        minVelocity.setConstant(0.0); // Default minimum velocities.
        pathArea = Eigen::MatrixXd(2, 4);
        pathArea.setZero(); // Initialize path area to zero.
        serpentinePolygonVertices.clear();
        homeLL = ctb::LatLong(homeGlobal(0), homeGlobal(1));    
        poseActualLL = ctb::LatLong(poseActualGlobal(0), poseActualGlobal(1));
        circularCenterLL = ctb::LatLong(44.096058, 9.864761); // Default circular center.
        circularStartPointLL = ctb::LatLong(44.096058, 9.864761); // Default circular start point.
        circularCenterLocal.setZero(); // Initialize circular center in local coordinates.
    }
};

} // namespace auv

#endif // AUV_CONTROL_DATA_STRUCTS_HPP
