#include "mission_ctrl/states/state_inspect_buoy.hpp"

namespace mission {

namespace states {

    StateInspectBuoy::StateInspectBuoy()
    {
    }

    StateInspectBuoy::~StateInspectBuoy()
    {
    }

    fsm::retval StateInspectBuoy::OnEntry()
    {
        debugCounter = 0;
        reachedInspectionPosition = false;
        sentDepthCmd = false;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "\nInspecting buoy...\n";
        }

        double distanceFromBuoy = 1.0; // 1m from the buoy
        // std::pair<std::string, Buoy> buoyToInspect;

        if (systemStatus_->conf.simPerception) {
            Eigen::Vector3d buoyPositionLocal = { 10.0, 0.0, ctrlData->depth }; // Simulated position in local NED coordinates
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, inspectionPosition, alt);
        } else {

            if (!GetBuoyToInspect()) {
                std::cerr << "No buoy to inspect found. Returning to searchBuoyArea state." << std::endl;
                return fsm_->SetNextState(states::ID::searchBuoyArea);
            }

            // Get point at 1m from the buoy in out direction
            inspectionPosition = buoyToInspect.position;
            std::cerr << "      Inspecting buoy at position: " << buoyToInspect.position.latitude << ", " << buoyToInspect.position.longitude << std::endl;
            Eigen::Vector3d buoyPositionLocal;
            ctb::LatLong2LocalNED(buoyToInspect.position, 0, ctrlData->inertialF_linearPosition, buoyPositionLocal);
            auto offset = -buoyPositionLocal.normalized() * distanceFromBuoy;
            buoyPositionLocal += offset;
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, inspectionPosition, alt);
            std::cerr << "          from position: " << inspectionPosition.latitude << ", " << inspectionPosition.longitude << std::endl;
        }
        // Set the goal position for the KCL command
        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = inspectionPosition.latitude;
        ctrlData->kclData.kclActionCmd.goal.position.longitude = inspectionPosition.longitude;
        ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthBuoys;

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::Execute()
    {
        std::cerr << "Executing state InspectBuoy: \n"
                  << "    - reachedInspectionPosition: " << reachedInspectionPosition
                  << "    - trying state: " << ctrlData->kclData.kclActionCmd.goal.desired_state << std::endl;
        if (!reachedInspectionPosition) {

            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, inspectionPosition, distance, azimuthRad);
            if (distance < systemStatus_->conf.latlongTolerance) {
                reachedInspectionPosition = true;
                inspectionStartTime = std::chrono::steady_clock::now();
                std::cerr << "Reached buoy and starting inspection." << std::endl;

                if (taskData_->taskType == taskBenchmarks::INSPECTION) {
                    std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
                    buoyActionColorMap = inspectionConf->buoysActions;
                } else if (taskData_->taskType == taskBenchmarks::INSPECTION_AND_INTERVENTION) {
                    std::shared_ptr<InspectionAndIntervention> inspectionConf = std::dynamic_pointer_cast<InspectionAndIntervention>(taskData_);
                    buoyActionColorMap = inspectionConf->buoysActions;
                } else {
                    this->SetNextMissionState();
                    return fsm::ok; // No buoys to search for
                }

                if (buoyToInspect.color == buoyActionColorMap.clockWiseRotationColor) {
                    std::cerr << "Buoy color is " << buoyToInspect.color << ", setting clockwise rotation." << std::endl;
                    ctrlData->kclData.kclActionCmd = mission::kclCmd();
                    ctrlData->kclData.kclActionCmd.goal.desired_state = "Circular2D";
                    ctrlData->kclData.kclActionCmd.goal.circular_data.circular_diameter = 2 * systemStatus_->conf.inspectBuoyOrbitingRadius;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.latitude = buoyToInspect.position.latitude;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.longitude = buoyToInspect.position.longitude;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.clockwise = true; // Clockwise rotation
                } else if (buoyToInspect.color == buoyActionColorMap.counterClockWiseRotationColor) {
                    std::cerr << "Buoy color is " << buoyToInspect.color << ", setting counter-clockwise rotation." << std::endl;
                    ctrlData->kclData.kclActionCmd = mission::kclCmd();
                    ctrlData->kclData.kclActionCmd.goal.desired_state = "Circular2D";
                    ctrlData->kclData.kclActionCmd.goal.circular_data.circular_diameter = 2 * systemStatus_->conf.inspectBuoyOrbitingRadius;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.latitude = buoyToInspect.position.latitude;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.longitude = buoyToInspect.position.longitude;
                    ctrlData->kclData.kclActionCmd.goal.circular_data.clockwise = false;
                } else if (buoyToInspect.color == buoyActionColorMap.goUpColor) {
                    std::cerr << "Buoy color is " << buoyToInspect.color << ", going up after 30s." << std::endl;

                } else if (buoyToInspect.color == buoyActionColorMap.goDownColor) {
                    std::cerr << "Buoy color is " << buoyToInspect.color << ", going down after 30s." << std::endl;

                } else {
                    std::cerr << "Unknown buoy color: " << buoyToInspect.color << ". Returning to searchBuoyArea state." << std::endl;
                    return fsm_->SetNextState(states::ID::searchBuoyArea);
                }
            }

            
        } else {
            // check progress
            auto elapsedTime = (std::chrono::steady_clock::now() - inspectionStartTime).count() / 1e9; // Convert to seconds
            if (buoyToInspect.color == buoyActionColorMap.clockWiseRotationColor || buoyToInspect.color == buoyActionColorMap.counterClockWiseRotationColor) {
                if (elapsedTime > systemStatus_->conf.inspectBuoyOrbitingTimeout) {
                    ctrlData->missionData.inspectedBuoys.push_back(buoyToInspect);
                    return fsm_->SetNextState(states::ID::searchBuoyArea);
                }
            } else if (!sentDepthCmd) {
                if (elapsedTime < 30.0)
                    return fsm::ok;

                sentDepthCmd = true;
                ctrlData->kclData.kclActionCmd = mission::kclCmd();

                if (buoyToInspect.color == buoyActionColorMap.goUpColor) {
                    ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
                    ctrlData->kclData.kclActionCmd.goal.position.latitude = ctrlData->inertialF_linearPosition.latitude;
                    ctrlData->kclData.kclActionCmd.goal.position.longitude = ctrlData->inertialF_linearPosition.longitude;
                    ctrlData->kclData.kclActionCmd.goal.depth = ctrlData->depth - 0.5;
                } else if (buoyToInspect.color == buoyActionColorMap.goDownColor) {
                    ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
                    ctrlData->kclData.kclActionCmd.goal.position.latitude = ctrlData->inertialF_linearPosition.latitude;
                    ctrlData->kclData.kclActionCmd.goal.position.longitude = ctrlData->inertialF_linearPosition.longitude;
                    ctrlData->kclData.kclActionCmd.goal.depth = ctrlData->depth + 0.5;
                }
                goalDepth = ctrlData->kclData.kclActionCmd.goal.depth; // Store the goal depth
            } else {
                // check depth
                double depthDifference = std::abs(ctrlData->depth - goalDepth);
                if (depthDifference < systemStatus_->conf.depthTolerance) {
                    ctrlData->missionData.inspectedBuoys.push_back(buoyToInspect);
                    std::cerr << "Reached goal depth of " << goalDepth << "m. Buoy inspection completed." << std::endl;
                    return fsm_->SetNextState(states::ID::searchBuoyArea);
                }
            }
        }

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::OnExit()
    {
        return fsm::ok;
    }

    bool StateInspectBuoy::GetBuoyToInspect()
    {
        std::vector<Buoy> buoysToInspect;

        // Get the buoy to inspect from the ctrlData
        for (const auto& db : ctrlData->perceptionData.detectedBuoys) {
            bool alreadyInspected = false;
            for (const auto& ib : ctrlData->missionData.inspectedBuoys) {
                if (db.second.detectionId == ib.detectionId) {
                    alreadyInspected = true;
                    break;
                }
            }
            if (!alreadyInspected) {
                // buoyToInspect = db;
                buoysToInspect.push_back(db.second);
                // buoyToInspect = db.second;
                break;
            }
        }

        // Get closest buoy to the current position
        if (buoysToInspect.empty()) {
            std::cerr << "No buoys to inspect found." << std::endl;
            return false;
        } else {
            double minDistance = std::numeric_limits<double>::max();
            for (const auto& buoy : buoysToInspect) {
                double distance, azimuthRad;
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, buoy.position, distance, azimuthRad);
                if (distance < minDistance) {
                    minDistance = distance;
                    buoyToInspect = buoy;
                }
            }
        }
        return true;
    }
}
}