#include "mission_ctrl/states/state_inspect_pipes.hpp"

namespace mission {

namespace states {

    StateInspectPipes::StateInspectPipes()
    {
    }

    StateInspectPipes::~StateInspectPipes()
    {
    }

    fsm::retval StateInspectPipes::OnEntry()
    {

        currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "Inspecting pipes...\n";
        }

        // tell kcl to spiral around the pipeline structure

        ctrlData->perceptionData.enableDtcPipes = true;
        // tell perception to detect pipes with code "" (no code)
        // ctrlData->perceptionData.currentPipeDtc.codeDtc = "";

        return fsm::ok;
    }

    fsm::retval StateInspectPipes::Execute()
    {
        std::cerr << ".";
        // Process perception data
        if (ctrlData->perceptionData.newDtcFromPerception) {
            ctrlData->perceptionData.newDtcFromPerception = false;
            auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;
            // Got the red marker
            if (ctrlData->perceptionData.currentPipeDtc.found_red_marker) {
                pipe.data.hasRedMarker = true;
                pipe.data.positionRedMarker.latlong.latitude = ctrlData->perceptionData.currentPipeDtc.red_marker_position.latlong.latitude;
                pipe.data.positionRedMarker.latlong.longitude = ctrlData->perceptionData.currentPipeDtc.red_marker_position.latlong.longitude;
                pipe.data.positionRedMarker.depth = ctrlData->perceptionData.currentPipeDtc.red_marker_position.depth;
            }
            // Got the pipe number
            if (ctrlData->perceptionData.currentPipeDtc.found_number) {
                pipe.data.foundPipeNumber = true;
                pipe.data.positionPipeNumber.latlong.latitude = ctrlData->perceptionData.currentPipeDtc.number_position.latlong.latitude;
                pipe.data.positionPipeNumber.latlong.longitude = ctrlData->perceptionData.currentPipeDtc.number_position.latlong.longitude;
                pipe.data.positionPipeNumber.depth = ctrlData->perceptionData.currentPipeDtc.number_position.depth;
                pipe.data.number = ctrlData->perceptionData.currentPipeDtc.number;
            }

            if (pipe.data.hasRedMarker && pipe.data.foundPipeNumber) {
                // Found everything.
            } else if (pipe.data.hasRedMarker) {
                // At least we know this it the damaged pipe.
            }
        }

        // Progress the inspection
        if (currentPhase == PipelinePipeInspectionPhase::LOOKING_FOR_PIPE) {
            // If we see a pipe, get close, otherwise keep following the path
            if (ctrlData->perceptionData.currentPipeDtc.pipe_in_fov) {
                // Found a pipe
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Pipe found: " << ctrlData->perceptionData.currentPipeDtc.pipeline_pipe_code << "\n";
                }
                onPipeInspection = true;
                currentPipeDtcCode = ctrlData->perceptionData.currentPipeDtc.pipeline_pipe_code;
                currentPhase = PipelinePipeInspectionPhase::MOVING_TO_POINT_ON_PIPE;
            }
        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_TO_POINT_ON_PIPE) {
            // Got a goal from dtc which should be on the pipe
            // -> check if reached the goal already
            // -> check if kcl is already moving to a point close to the new one

            ctb::LatLong pointOnPipe;
            pointOnPipe.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
            pointOnPipe.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;

            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pointOnPipe, distance, azimuthRad);
            if (distance < 0.5) {
                // Point reached
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Reached point on pipe: (" << pointOnPipe.latitude << ", " << pointOnPipe.longitude << ")\n";
                }
                currentPhase = PipelinePipeInspectionPhase::MOVING_AWAY_FROM_PIPELINE_STRUCTURE;
                return fsm::ok;
            }

            ctb::LatLong currentGoal;
            currentGoal.latitude = ctrlData->kclData.new_kcl_command.position.latitude;
            currentGoal.longitude = ctrlData->kclData.new_kcl_command.position.longitude;
            ctb::DistanceAndAzimuthRad(currentGoal, pointOnPipe, distance, azimuthRad);
            if (ctrlData->kclData.executingCommand && distance < 0.5) {
                return fsm::ok;
            }

            // tell kcl to move to point on pipe and align to direction
            ctrlData->perceptionData.currentPipeDtc.pipe_direction;
            ctrlData->kclData.newCommand = true;
            ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.new_kcl_command.position.latitude = pointOnPipe.latitude;
            ctrlData->kclData.new_kcl_command.position.longitude = pointOnPipe.longitude;
            ctrlData->kclData.new_kcl_command.depth = taskData_->diveDepth;

        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_AWAY_FROM_PIPELINE_STRUCTURE) {
            // We are close to the structure, so we have to move away from it
            // until dtc tells us to move to the structure
            if (ctrlData->perceptionData.currentPipeDtc.move_toward_structure) {
                currentPhase = PipelinePipeInspectionPhase::MOVING_TO_PIPELINE_STRUCTURE;
                auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;
                // Save the end position of the pipe
                pipe.endPosition.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
                pipe.endPosition.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;
            } else {
                // tell kcl to move away from the pipeline structure
                ctrlData->perceptionData.currentPipeDtc.vertical_distance; // has to be like 0.3m
            }

        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_TO_PIPELINE_STRUCTURE) {
            // We are far from the structure, so we have to move toward it
            // until dtc tells us to stop by setting move_toward_structure to false
            if (!ctrlData->perceptionData.currentPipeDtc.move_toward_structure) {
                // reached the structure
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Reached pipeline structure\n";
                }
                // Save the start position of the pipe
                auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;
                pipe.startPosition.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
                pipe.startPosition.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;
                
                pipe.numberOfLaps++;
                currentPhase = PipelinePipeInspectionPhase::ADDITIONAL_INSPECTION_LAP;
                // Maybe cleaner to ask the KCL to compute a path of n laps over the pipe
                if (pipe.numberOfLaps >= maxInspectionlaps) {
                    onPipeInspection = false;
                    auto& otherPipe = (currentPipeDtcCode == "A") ? pipeB : pipeA;
                    if (otherPipe.numberOfLaps < maxInspectionlaps) {
                        // tell dtc to inspect the other pipe
                        // tell kcl to resume spiral
                        currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;
                    } else {
                        // FAIL
                    }
                }
                return fsm::ok;
            } else {
                // tell kcl to move to the pipeline structure
                ctrlData->perceptionData.currentPipeDtc.vertical_distance; // has to be like 0.3m
            }

        } else if (currentPhase == PipelinePipeInspectionPhase::ADDITIONAL_INSPECTION_LAP) {
            auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;

            ctb::LatLong targetPosition;
            double distance, azimuthRad;
            if (pipe.movingToStructure) {
                // Check if arrived to start
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pipe.startPosition, distance, azimuthRad);
                if (distance < 0.5) {
                    pipe.numberOfLaps++;
                } else {
                    targetPosition = pipe.startPosition;
                    // Tell kcl to go to targetPosition
                }
            } else {
                // Check if arrived to end
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pipe.endPosition, distance, azimuthRad);
                if (distance < 0.5) {
                    pipe.numberOfLaps++;
                    pipe.movingToStructure = true;
                } else {
                    targetPosition = pipe.endPosition;
                    // Tell kcl to go to targetPosition
                }
            }
            if (pipe.numberOfLaps >= maxInspectionlaps) {
                onPipeInspection = false;
                auto& otherPipe = (currentPipeDtcCode == "A") ? pipeB : pipeA;
                if (otherPipe.numberOfLaps < maxInspectionlaps) {
                    // tell dtc to inspect the other pipe
                    // tell kcl to resume spiral
                    currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;
                } else {
                    // FAIL
                }
            }
        }

        return fsm::ok;
    }

    fsm::retval StateInspectPipes::OnExit()
    {

        return fsm::ok;
    }
}
}
