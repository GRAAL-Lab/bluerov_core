#include "mission_ctrl/states/state_cross_gate.hpp"

namespace mission {

namespace states {

    StateCrossGate::StateCrossGate()
    {
    }

    StateCrossGate::~StateCrossGate() { }

    fsm::retval StateCrossGate::OnEntry()
    {
        reachedFrontOfGate = false;
        if (systemStatus_->conf.simPerception) {
            ctb::LatLong posA, posB;
            if (systemStatus_->conf.debugBuoysPositions.size() < 2) {
                Eigen::Vector3d localPosA(5, -1, 0);
                Eigen::Vector3d localPosB(5, 1, 0);
                double alt;
                ctb::LocalNED2LatLong(localPosA, ctrlData->inertialF_linearPosition, posA, alt);
                ctb::LocalNED2LatLong(localPosB, ctrlData->inertialF_linearPosition, posB, alt);
            } else {

                posA = systemStatus_->conf.debugBuoysPositions[0];
                posB = systemStatus_->conf.debugBuoysPositions[1];
                std::cerr << "Using debug buoys positions for gate simulation: "
                          << posA.latitude << ", " << posA.longitude << " and "
                          << posB.latitude << ", " << posB.longitude << "\n";
            }
            Buoy gb1, gb2;
            Gate gt;
            gb1.position = posA;
            gb2.position = posB;

            gt.SetGateBuoys(gb1, gb2, true);

            ctrlData->missionData.gate = gt;

            std::cerr << "Buoys simulated to be at: " << gb1.position.latitude << ", "
                      << gb1.position.longitude << " and " << gb2.position.latitude << ", "
                      << gb2.position.longitude << "\n";
        }

        systemStatus_->conf.gateWpsDistance;

        Eigen::Vector3d buoy1_buoy2Pos;
        ctb::LatLong2LocalNED(ctrlData->missionData.gate.buoy2.position, 0, ctrlData->missionData.gate.buoy1.position, buoy1_buoy2Pos);
        Eigen::Vector3d perpendicularVector = systemStatus_->conf.gateWpsDistance * Eigen::Vector3d(-buoy1_buoy2Pos.y(), buoy1_buoy2Pos.x(), 0).normalized();
        Eigen::Vector3d firstWpLocal = buoy1_buoy2Pos.normalized() * (buoy1_buoy2Pos.norm() / 2) + perpendicularVector;
        Eigen::Vector3d secondWpLocal = buoy1_buoy2Pos.normalized() * (buoy1_buoy2Pos.norm() / 2) - perpendicularVector;
        double alt;
        ctb::LocalNED2LatLong(firstWpLocal, ctrlData->missionData.gate.buoy1.position, firstWp, alt);
        ctb::LocalNED2LatLong(secondWpLocal, ctrlData->missionData.gate.buoy1.position, secondWp, alt);

        double dist1, dist2, azimuthRad;
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, ctrlData->missionData.gate.buoy1.position, dist1, azimuthRad);
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, ctrlData->missionData.gate.buoy2.position, dist2, azimuthRad);
        if (dist1 > dist2) {
            ctb::LatLong temp = firstWp;
            firstWp = secondWp;
            secondWp = temp;
        }
        return fsm::ok;
    }

    fsm::retval StateCrossGate::Execute()
    {

        double distance, azimuthRad;
        if (!reachedFrontOfGate) {
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, firstWp, distance, azimuthRad);
            if (distance < systemStatus_->conf.latlongTolerance) {
                reachedFrontOfGate = true;
                ctrlData->kclData.kclActionCmd = mission::kclCmd();

                // ctrlData->kclData.kclActionCmd.underExecution = false; // Reset the command execution state
            } // }else{
            //     std::cerr << "Distance to first waypoint: "<< distance << "m which is at "
            //               << firstWp.latitude << ", " << firstWp.longitude << "\n";
            // }
        } else {
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, secondWp, distance, azimuthRad);
            if (distance < systemStatus_->conf.latlongTolerance) {
                return this->SetNextMissionState();
            } // }else{
            //     std::cerr << "Distance to second waypoint: " << distance << "m which is at "
            //               << secondWp.latitude << ", " << secondWp.longitude << "\n";

            // }
        }
        if (ctrlData->kclData.kclActionCmd.feedback.actual_state != "WAYPOINT_NAVIGATION") {
            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
            if (reachedFrontOfGate) {
                if (systemStatus_->conf.debugPrints)
                    std::cerr << "Reached front of gate, moving to second waypoint: "
                              << secondWp.latitude << ", " << secondWp.longitude << "\n";

                ctrlData->kclData.kclActionCmd.goal.position.latitude = secondWp.latitude;
                ctrlData->kclData.kclActionCmd.goal.position.longitude = secondWp.longitude;
                ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthGate;
            } else {
                ctrlData->kclData.kclActionCmd.goal.position.latitude = firstWp.latitude;
                ctrlData->kclData.kclActionCmd.goal.position.longitude = firstWp.longitude;
                ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthGate;
            }
        }

        return fsm::ok;
    }

    fsm::retval StateCrossGate::OnExit()
    {
        return fsm::ok;
    }

}
}
