#include "mission_ctrl/states/state_search_object.hpp"

namespace mission
{

    namespace states
    {

        StateSearchObject::StateSearchObject()
        {
        }

        StateSearchObject::~StateSearchObject() {}

        fsm::retval StateSearchObject::OnEntry()
        {
            reachedLeftmostPoint = false;
            sentPathFollowingCommand = false;
            return StartSearch();
        }

        fsm::retval StateSearchObject::Execute()
        {

            if (!systemStatus_->conf.simPerception && ctrlData->perceptionData.newDtcFromPerception)
            {
                ctrlData->perceptionData.newDtcFromPerception = false;
                if (taskData_->taskPhases.front().second == opis::gate)
                {
                    for (auto &db_first : ctrlData->perceptionData.detectedBuoys)
                    {
                        for (auto &db_second : ctrlData->perceptionData.detectedBuoys)
                        {
                            if (db_first.second.detectionId == db_second.second.detectionId)
                            {
                                continue; // same buoy
                            }
                            if (ctrlData->missionData.gate.SetGateBuoys(db_first.second, db_second.second, systemStatus_->conf.ignoreBuoyColor))
                            {
                                // Found the gate
                                if (systemStatus_->conf.debugPrints)
                                {
                                    std::cerr << "Gate found: " << db_first.second.detectionId << " and " << db_second.second.detectionId << "\n";
                                }
                                ctrlData->missionData.foundGate = true;
                                return SetNextMissionState();
                            }
                        }
                    }
                }
                else if (taskData_->taskPhases.front().second == opis::mainPipe)
                {
                }
                else if (taskData_->taskPhases.front().second == opis::manipulationConsole)
                {
                    if (ctrlData->perceptionData.dtcList.manipulation_console)
                    {
                        if (systemStatus_->conf.debugPrints)
                        {
                            std::cerr << "Manipulation console FOUND!!! \n";
                        }
                        return SetNextMissionState();
                    }
                }
            }

            // if (!doingSerpentine)
            //     return fsm::ok;
                
            if (!reachedLeftmostPoint)
            {
                double distance, azimuthRad;
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, areaPoints.front(), distance, azimuthRad);
                if (distance < systemStatus_->conf.latlongTolerance)
                {
                    reachedLeftmostPoint = true;
                }
                return fsm::ok;
            }

            if (reachedLeftmostPoint && !sentPathFollowingCommand)
            {
                sentPathFollowingCommand = true;

                // tell kcl to follow area coverage path
                ctrlData->kclData.kclActionCmd = mission::kclCmd();

                ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
                ctrlData->kclData.kclActionCmd.goal.path_mode = "Serpentine2D";

                auto points = areaPoints;

                ctrlData->kclData.kclActionCmd.goal.serpentine_data.origin.latitude = points.front().latitude;
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.origin.longitude = points.front().longitude;
                points.pop();
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_left.longitude = points.front().longitude;
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_left.latitude = points.front().latitude;
                points.pop();
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_right.longitude = points.front().longitude;
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_right.latitude = points.front().latitude;
                points.pop();
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.right.longitude = points.front().longitude;
                ctrlData->kclData.kclActionCmd.goal.serpentine_data.right.latitude = points.front().latitude;

                return fsm::ok;
            }
            // if (ctrlData->kclData.kclActionCmd.feedback.actual_state != "PATH_FOLLOWING") {
            //     return StartSearch();
            // }

            return fsm::ok;
        }

        fsm::retval StateSearchObject::OnExit()
        {
            ctrlData->perceptionData.enableDtcBuoys = false;
            ctrlData->perceptionData.enableDtcManipulationConsole = false;
            ctrlData->perceptionData.desiredGimbalAttitude = 0.0;

            return fsm::ok;
        }

        fsm::retval StateSearchObject::StartSearch()
        {
            areaPoints = std::queue<ctb::LatLong>();

            // Ordering points for the area coverage path (depening on current position)
            auto points = taskData_->buoysArea.points;
            auto removePoint = [&points](const ctb::LatLong &point)
            {
                points.erase(
                    std::remove_if(points.begin(), points.end(),
                                   [&point](const ctb::LatLong &p)
                                   {
                                       return p.latitude == point.latitude && p.longitude == point.longitude;
                                   }),
                    points.end());
            };
            auto findLeftmostPoint = [&](const ctb::LatLong &reference) -> std::optional<ctb::LatLong>
            {
                if (points.empty())
                    return std::nullopt;
                ctb::LatLong leftmostPoint = points.front();
                double minAzimuthRad = M_PI;
                for (const auto &point : points)
                {
                    double distance, azimuthRad;
                    ctb::DistanceAndAzimuthRad(reference, point, distance, azimuthRad);
                    if (azimuthRad < minAzimuthRad)
                    {
                        minAzimuthRad = azimuthRad;
                        leftmostPoint = point;
                    }
                }
                return leftmostPoint;
            };
            for (size_t i = 0; i < taskData_->buoysArea.points.size(); ++i)
            {
                auto maybePoint = findLeftmostPoint(ctrlData->inertialF_linearPosition);
                if (!maybePoint)
                    break; // no more points
                areaPoints.push(*maybePoint);
                removePoint(*maybePoint);
            }

            if (taskData_->taskPhases.front().second == opis::gate)
            {
                std::cerr << "Searching for gate...\n";
                ctrlData->missionData.foundGate = false;
                ctrlData->perceptionData.enableDtcBuoys = true;

                // ctrlData->kclData.kclActionCmd = mission::kclCmd();
                // ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
                // ctrlData->kclData.kclActionCmd.goal.path_mode = "Spiral2D";
                // ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_diameter = 5.0;
                // ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_increment = 0.5;

                // Move to leftmost point
                //doingSerpentine = true;
                ctrlData->kclData.kclActionCmd = mission::kclCmd();
                ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
                ctrlData->kclData.kclActionCmd.goal.position.latitude = areaPoints.front().latitude;
                ctrlData->kclData.kclActionCmd.goal.position.longitude = areaPoints.front().longitude;
                ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthGate;
            }
            else if (taskData_->taskPhases.front().second == opis::mainPipe)
            {
                std::cerr << "Searching for main pipe...\n";

                // tell the KCL to turn around
            }
            else if (taskData_->taskPhases.front().second == opis::manipulationConsole)
            {
                std::cerr << "Searching for manipulation console...\n";

                ctrlData->perceptionData.enableDtcManipulationConsole = true;
                ctrlData->perceptionData.desiredGimbalAttitude = 45.0 / 180.0 * M_PI; // 50 degrees max

                // ctrlData->kclData.kclActionCmd = mission::kclCmd();
                // ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
                // ctrlData->kclData.kclActionCmd.goal.path_mode = "Spiral2D";
                // ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_diameter = 5.0;
                // ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_increment = 0.5;

                //doingSerpentine = true;
                reachedLeftmostPoint = true;
                // ctrlData->kclData.kclActionCmd = mission::kclCmd();
                // ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
                // ctrlData->kclData.kclActionCmd.goal.position.latitude = areaPoints.front().latitude;
                // ctrlData->kclData.kclActionCmd.goal.position.longitude = areaPoints.front().longitude;
                // ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthManipulationConsole;
            }
            else
            {
                return fsm::fail;
            }

            return fsm::ok;
        }
    }
}
