#include "mission_ctrl/states/state_depth.hpp"

namespace mission {

namespace states {

    StateDepth::StateDepth()
    {
    }

    StateDepth::~StateDepth() { }

    fsm::retval StateDepth::OnEntry()
    {

        if (taskData_->taskPhases.front().second == "BUOYS_SEARCH") {
            goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
            goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
            goalPose.at(2) = systemStatus_->conf.diveDepthBuoys;
        } else if (taskData_->taskPhases.front().second == "GATE_SEARCH") {

            goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
            goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
            goalPose.at(2) = systemStatus_->conf.diveDepthGate;
        } else if (taskData_->taskPhases.front().second == "MANIPULATION_CONSOLE_SEARCH") {
            goalPose.at(0) = systemStatus_->conf.debugPositions[0].latitude;
            goalPose.at(1) = systemStatus_->conf.debugPositions[0].longitude;
            goalPose.at(2) = systemStatus_->conf.diveDepthManipulationConsole;
        } else {
            std::cerr << "[WARNING] No task phase set for StateDepth, using default values.\n";
            goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
            goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
            goalPose.at(2) = systemStatus_->conf.diveDepth;
        }

        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = goalPose.at(0);
        ctrlData->kclData.kclActionCmd.goal.position.longitude = goalPose.at(1);
        ctrlData->kclData.kclActionCmd.goal.depth = goalPose.at(2);

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "Moving from LatLong: ("
                      << ctrlData->inertialF_linearPosition.latitude << ", "
                      << ctrlData->inertialF_linearPosition.longitude << ") and Depth: "
                      << ctrlData->depth << "\n";
            std::cerr << "         to LatLong: ("
                      << goalPose.at(0) << ", "
                      << goalPose.at(1) << ") and Depth: "
                      << goalPose.at(2) << "\n";
        }

        return fsm::ok;
    }

    fsm::retval StateDepth::Execute()
    {

        if (abs(ctrlData->depth - goalPose.at(2)) < systemStatus_->conf.depthTolerance) {
            return this->SetNextMissionState();
        }

        return fsm::ok;
    }

    fsm::retval StateDepth::OnExit()
    {

        return fsm::ok;
    }
}
}
