#pragma once

// -------------------------
// Standard Library Headers
// -------------------------
#include <string>
#include <memory>

// -------------------------
// Third-Party Library Headers
// -------------------------
#include <Eigen/Dense>
#include <fsm/fsm.h>
#include <chrono>

// -------------------------
// ROS 2 Headers
// -------------------------
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

// -------------------------
// Project-Specific Headers
// -------------------------
#include "kcl/data_structs.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "auv_core_helper/srv/control_command.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "states/commands.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "rml/EulerRPY.h"

// -------------------------
// Base State Class for AUV FSM
// -------------------------

/// The `BaseAUVState` class serves as a base class for all states in the FSM controlling an Autonomous Underwater Vehicle (AUV).
/// Derived states must implement the `OnEntry`, `Execute`, and `OnExit` methods to define their behavior.
class BaseAUVState : public fsm::BaseState {
protected:
    fsm::FSM* fsm_; ///< Pointer to the FSM instance controlling this state.
    std::string stateName_; ///< Name of the state, primarily for debugging/logging.

public:
    std::shared_ptr<auv::ControlData> ctrlData; ///< Shared pointer to control data used by the state.

    /// Constructor for the `BaseAUVState` class.
    /// @param fsm Pointer to the FSM instance.
    /// @param name Name of the state.
    BaseAUVState(fsm::FSM* fsm, const std::string& name);

    /// Virtual destructor to ensure proper cleanup in derived classes.
    virtual ~BaseAUVState() = default;

    /// Called when entering the state. Must be implemented by derived classes.
    /// @return Return value indicating the result of entering the state.
    virtual fsm::retval OnEntry() = 0;

    /// Called repeatedly while in the state. Must be implemented by derived classes.
    /// @return Return value indicating the result of the execution.
    virtual fsm::retval Execute() = 0;

    /// Called when exiting the state. Must be implemented by derived classes.
    /// @return Return value indicating the result of exiting the state.
    virtual fsm::retval OnExit() = 0;
};
