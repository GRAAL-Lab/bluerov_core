#ifndef MISSION_CTRL_DATA_STRUCTS_HPP
#define MISSION_CTRL_DATA_STRUCTS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>
#include <queue>

#include "ctrl_toolbox/HelperFunctions.h"
#include "mission_ctrl/mission_ctrl_defines.hpp"
#include "rclcpp/rclcpp.hpp"

#include "auv_core_helper/action/set_kcl.hpp"
#include "auv_core_helper/srv/mission_command.hpp"

namespace mission {

// === Forward declarations ===

struct DtcBuoy;
struct GateBuoy;
struct PerceptionData;
struct KinematicData;
struct SystemStatus;
struct ControlData;

struct Buoy;
struct Gate;
struct BuoysArea;
struct PipelinePipe;
struct PipelineStructure;

struct TaskBenchmarkSettings;
struct Inspection;
struct Intervention;
struct InspectionAndIntervention;

// ===========================

struct BuoyActionColorMap {
    std::string clockWiseRotationColor;
    std::string counterClockWiseRotationColor;
    std::string goUpColor;
    std::string goDownColor;
    friend std::ostream& operator<<(std::ostream& os, BuoyActionColorMap const& map)
    {
        os << "BuoyActionColorMap {\n";
        os << "  ClockWiseRotation: " << map.clockWiseRotationColor << "\n";
        os << "  CounterClockWiseRotation: " << map.counterClockWiseRotationColor << "\n";
        os << "  GoUp: " << map.goUpColor << "\n";
        os << "  GoDown: " << map.goDownColor << "\n";
        os << "}\n";
        return os;
    }
};

struct Buoy {
    std::string detectionId;
    ctb::LatLong position;
    double radius; // 0.1m for gate buoys, 0.15m for dtc buoys
    std::string color;
    double colorConfidence;
};

struct Gate {
    Buoy buoy1;
    Buoy buoy2;
    double distanceTolerance = 0.5;
    double expectedDistance = 2.0;

    bool SetGateBuoys(const Buoy& b1, const Buoy& b2)
    {
        Eigen::Vector3d distanceVector;
        ctb::LatLong2LocalNED(b1.position, 0, b2.position, distanceVector);
        if (distanceVector.norm() > expectedDistance + distanceTolerance || distanceVector.norm() < expectedDistance - distanceTolerance) {
            return false;
        }
        buoy1 = b1;
        buoy2 = b2;
        return true;
    }
};

struct MissionData {
    std::vector<Buoy> inspectedBuoys;
    Gate gate;
};

struct PerceptionData {
    bool isAlive;
    std::string state;

    bool enableDtcObstacles;
    bool enableDtcBuoys;

    std::map<std::string, Buoy> detectedBuoys;
};

struct KinematicData {
    bool isAlive;
    std::string state;

    bool newCommand;
    bool executingCommand;
    auv_core_helper::action::SetKCL::Goal kcl_command;
};

struct ControlData {
    ctb::LatLong inertialF_linearPosition;
    double depth;
    rml::EulerRPY bodyF_angularPosition;

    KinematicData kclData;
    PerceptionData perceptionData;
    MissionData missionData;
};

struct BuoysArea {
    bool enabled = false;
    ctb::LatLong centroid;
    std::vector<double> size;
    double orientation;

    friend std::ostream& operator<<(std::ostream& os, BuoysArea const& area)
    {
        os << "BuoysArea {\n";
        os << "  enabled: " << std::boolalpha << area.enabled << "\n";
        os << "  centroid: (" << area.centroid.latitude << ", " << area.centroid.longitude << ")\n";
        os << "  size: [";
        for (size_t i = 0; i < area.size.size(); ++i) {
            os << area.size[i];
            if (i != area.size.size() - 1)
                os << ", ";
        }
        os << "]\n";
        os << "  orientation: " << area.orientation << "\n";
        os << "}\n";
        return os;
    }
};

struct PipelinePipe {
    uint number;
    double orientation;
    ctb::LatLong position;

    friend std::ostream& operator<<(std::ostream& os, const PipelinePipe& pipe)
    {
        os << "PipelinePipe {\n";
        os << "  number: " << pipe.number << "\n";
        os << "  orientation: " << pipe.orientation << "\n";
        os << "  position: (" << pipe.position.latitude << ", " << pipe.position.longitude << ")\n";
        os << "}\n";
        return os;
    }
};
struct PipelineStructure {
    uint id;
    ctb::LatLong centroid;

    friend std::ostream& operator<<(std::ostream& os, const PipelineStructure& structure)
    {
        os << "PipelineStructure {\n";
        os << "  id: " << structure.id << "\n";
        os << "  centroid: (" << structure.centroid.latitude << ", " << structure.centroid.longitude << ")\n";
        os << "}\n";
        return os;
    }
};

struct TaskBenchmarkSettings {
    std::string taskType;
    std::queue<std::pair<std::string, std::string>> taskPhases;

    std::vector<PipelineStructure> pipelineStructures;
    uint selectedPipelineStructureId;
    BuoysArea buoysArea;
    double surfaceDepth = 0.0;
    double diveDepth = 1.0;

    TaskBenchmarkSettings() = default;

    virtual bool ConfigureFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request)
    {
        try {
            auto pipelineStructures = request->pipeline_structures;
            for (size_t i = 0; i < pipelineStructures.size(); ++i) {
                PipelineStructure pStruct;
                pStruct.id = static_cast<uint>(i + 1);
                pStruct.centroid.latitude = pipelineStructures[i].latitude;
                pStruct.centroid.longitude = pipelineStructures[i].longitude;
                this->pipelineStructures.push_back(pStruct);
            }
            this->selectedPipelineStructureId = request->selected_pipeline_structure_id;
            this->buoysArea.enabled = true;
            this->buoysArea.centroid.latitude = request->buoys_area.centroid.latitude;
            this->buoysArea.centroid.longitude = request->buoys_area.centroid.longitude;
            this->buoysArea.size.push_back(request->buoys_area.size[0]);
            this->buoysArea.size.push_back(request->buoys_area.size[1]);
            this->buoysArea.orientation = request->buoys_area.orientation;
        } catch (...) {
            return false;
        }
        return true;
    }

    virtual bool ConfigureFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& pipelineStructuresSetting = root["pipelineStructures"];
        for (int i = 0; i < pipelineStructuresSetting.getLength(); ++i) {
            const libconfig::Setting& pipelineStructure = pipelineStructuresSetting[i];
            PipelineStructure pStruct;
            pStruct.id = static_cast<uint>(i + 1);
            if (!LatLongFromConfig(pipelineStructure, pStruct.centroid, "centroid")) {
                std::cerr << "Failed to load centroid from file" << std::endl;
                return false;
            };
            pipelineStructures.push_back(pStruct);
        }
        if (!ctb::GetParam(confObj, selectedPipelineStructureId, "selectedPipelineStructureId"))
            return false;

        if (!ctb::GetParam(confObj, buoysArea.enabled, "enableBuoysArea"))
            return false;
        if (buoysArea.enabled) {
            if (!LatLongFromConfig(root, buoysArea.centroid, "buoysAreaCentroid"))
                return false;
            Eigen::VectorXd sizeTmp;
            if (!ctb::GetParamVector(confObj, sizeTmp, "buoysAreaSize"))
                return false;
            buoysArea.size.push_back(sizeTmp[0]);
            buoysArea.size.push_back(sizeTmp[1]);
            if (!ctb::GetParam(root, buoysArea.orientation, "buoysAreaOrientation"))
                return false;
        }

        return true;
    }

protected:
    bool LatLongFromConfig(const libconfig::Setting& confObj, ctb::LatLong& latLong, const std::string& paramName)
    {
        Eigen::VectorXd latLongTmp;
        if (!ctb::GetParamVector(confObj, latLongTmp, paramName))
            return false;
        latLong.latitude = latLongTmp[0];
        latLong.longitude = latLongTmp[1];
        return true;
    }

    bool GetPipelinePipesFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request, std::vector<PipelinePipe>& pPipes)
    {
        try {
            auto pipelinePipes = request->pipes;
            for (size_t i = 0; i < pipelinePipes.size(); ++i) {
                PipelinePipe pipe;
                pipe.number = static_cast<uint>(i + 1);
                pipe.orientation = pipelinePipes[i].orientation;
                pipe.position.latitude = pipelinePipes[i].centroid.latitude;
                pipe.position.longitude = pipelinePipes[i].centroid.longitude;
                pPipes.push_back(pipe);
            }
        } catch (...) {
            return false;
        }
        return true;
    }

    bool GetPipelinePipesFromFile(libconfig::Config& confObj, std::vector<PipelinePipe>& pipelinePipes)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& pipelinePipesSetting = root["pipelinePipes"];
        for (int i = 0; i < pipelinePipesSetting.getLength(); ++i) {
            const libconfig::Setting& pipelinePipeSetting = pipelinePipesSetting[i];
            PipelinePipe pipe;
            if (!ctb::GetParam(pipelinePipeSetting, pipe.number, "number"))
                return false;
            if (!ctb::GetParam(pipelinePipeSetting, pipe.orientation, "orientation"))
                return false;
            if (!LatLongFromConfig(pipelinePipeSetting, pipe.position, "centroid"))
                return false;
            pipelinePipes.push_back(pipe);
        }
        return true;
    }

    bool GetBuoysActionsFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request, BuoyActionColorMap& buoysActionsMap)
    {
        try {
            buoysActionsMap.clockWiseRotationColor = request->buoys_action.color_clockwise_rotation;
            buoysActionsMap.counterClockWiseRotationColor = request->buoys_action.color_counterclockwise_rotation;
            buoysActionsMap.goUpColor = request->buoys_action.color_go_up;
            buoysActionsMap.goDownColor = request->buoys_action.color_go_down;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool GetBuoysActionsFromFile(libconfig::Config& confObj, BuoyActionColorMap& buoysActionsMap)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& buoysActions = root["buoysActions"];
        std::string color;
        if (!ctb::GetParam(buoysActions, color, "clockWiseRotation"))
            return false;
        buoysActionsMap.clockWiseRotationColor = color;
        // buoysActionsMap.emplace(ClockWiseRotation, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "counterClockWiseRotation"))
            return false;
        buoysActionsMap.counterClockWiseRotationColor = color;
        // buoysActionsMap.emplace(CounterClockWiseRotation, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "goUp"))
            return false;
        buoysActionsMap.goUpColor = color;
        // buoysActionsMap.emplace(GoUp, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "goDown"))
            return false;
        buoysActionsMap.goDownColor = color;
        // buoysActionsMap.emplace(GoDown, static_cast<BuoyColor>(color));
        return true;
    }

    virtual void dump(std::ostream& os) const
    {
        os << "PipelineStructures:\n";
        for (auto const& ps : pipelineStructures)
            os << ps;
        os << "SelectedPipelineStructureId: " << selectedPipelineStructureId << "\n";
        os << "BuoysArea:\n"
           << buoysArea;
    }

    // 2) Make operator<< non‐overload, always dispatch via dump()
    friend std::ostream& operator<<(std::ostream& os, TaskBenchmarkSettings const& s)
    {
        s.dump(os);
        return os;
    }
};

struct Inspection : public TaskBenchmarkSettings {
    ctb::LatLong uavWaypoint;
    uint numberOfBuoys;
    BuoyActionColorMap buoysActions;
    std::vector<PipelinePipe> pipelinePipes;

    Inspection()
    {
        taskType = taskBenchmarks::INSPECTION;
    }

    bool ConfigureFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromSrv(request))
            return false;
        try {
            uavWaypoint.latitude = request->uav_wp.latitude;
            uavWaypoint.longitude = request->uav_wp.longitude;
            numberOfBuoys = request->n_buoys;
            if (!GetBuoysActionsFromSrv(request, buoysActions))
                return false;
            if (!GetPipelinePipesFromSrv(request, pipelinePipes))
                return false;
        } catch (...) {
            return false;
        }
        return true;
    }

    bool ConfigureFromFile(libconfig::Config& confObj) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromFile(confObj))
            return false;
        const libconfig::Setting& root = confObj.getRoot();
        if (!LatLongFromConfig(root, uavWaypoint, "uavWaypoint"))
            return false;
        if (!ctb::GetParam(confObj, numberOfBuoys, "numberOfBuoys"))
            return false;
        if (!GetBuoysActionsFromFile(confObj, buoysActions))
            return false;
        if (!GetPipelinePipesFromFile(confObj, pipelinePipes))
            return false;
        return true;
    }

    void dump(std::ostream& os) const override
    {
        TaskBenchmarkSettings::dump(os);
        os << "\n";
        os << "UavWaypoint: (" << uavWaypoint.latitude << ", " << uavWaypoint.longitude << ")\n";
        os << "NumberOfBuoys: " << numberOfBuoys << "\n";
        os << buoysActions;
        os << "PipelinePipes:\n";
        for (auto const& p : pipelinePipes)
            os << p;
        os << "===========================\n";
    }
};

struct Intervention : public TaskBenchmarkSettings {
    uint numberOfMainPipeDamageMarkers;
    PipelinePipe damagedPipeOnPipeline;

    Intervention()
    {
        taskType = taskBenchmarks::INTERVENTION;
    }

    bool ConfigureFromFile(libconfig::Config& confObj) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromFile(confObj))
            return false; // Call the base class code first!
        if (!ctb::GetParam(confObj, numberOfMainPipeDamageMarkers, "numberOfMainPipeDamageMarkers"))
            return false;

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& pipelinePipeSetting = root["damagedPipeOnPipeline"];
        if (!ctb::GetParam(pipelinePipeSetting, damagedPipeOnPipeline.number, "number"))
            return false;
        if (!ctb::GetParam(pipelinePipeSetting, damagedPipeOnPipeline.orientation, "orientation"))
            return false;
        if (!LatLongFromConfig(pipelinePipeSetting, damagedPipeOnPipeline.position, "centroid"))
            return false;

        return true;
    }

    bool ConfigureFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromSrv(request))
            return false;
        try {
            numberOfMainPipeDamageMarkers = request->n_damage_markers;
            damagedPipeOnPipeline.number = request->damaged_pipe.number;
            damagedPipeOnPipeline.orientation = request->damaged_pipe.orientation;
            damagedPipeOnPipeline.position.latitude = request->damaged_pipe.centroid.latitude;
            damagedPipeOnPipeline.position.longitude = request->damaged_pipe.centroid.longitude;
        } catch (...) {
            return false;
        }
        return true;
    }

    void dump(std::ostream& os) const override
    {
        TaskBenchmarkSettings::dump(os);
        os << "\n";
        os << "NumberOfMainPipeDamageMarkers: " << numberOfMainPipeDamageMarkers << "\n";
        os << "DamagedPipeOnPipeline:\n"
           << damagedPipeOnPipeline;
        os << "=============================\n";
    }
};

struct InspectionAndIntervention : public TaskBenchmarkSettings {
    uint numberOfMainPipeDamageMarkers;
    uint numberOfBuoys;
    BuoyActionColorMap buoysActions;
    std::vector<PipelinePipe> pipelinePipes;

    InspectionAndIntervention()
    {
        taskType = taskBenchmarks::INSPECTION_AND_INTERVENTION;
    }

    bool ConfigureFromFile(libconfig::Config& confObj) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromFile(confObj))
            return false; // Call the base class code first!
        if (!ctb::GetParam(confObj, numberOfMainPipeDamageMarkers, "numberOfMainPipeDamageMarkers"))
            return false;
        if (!ctb::GetParam(confObj, numberOfBuoys, "numberOfBuoys"))
            return false;
        if (!GetBuoysActionsFromFile(confObj, buoysActions))
            return false;
        if (!GetPipelinePipesFromFile(confObj, pipelinePipes))
            return false;
        return true;
    }

    bool ConfigureFromSrv(const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromSrv(request))
            return false;
        try {
            numberOfMainPipeDamageMarkers = request->n_damage_markers;
            numberOfBuoys = request->n_buoys;
            if (!GetBuoysActionsFromSrv(request, buoysActions))
                return false;
            if (!GetPipelinePipesFromSrv(request, pipelinePipes))
                return false;
        } catch (...) {
            return false;
        }
        return true;
    }

    void dump(std::ostream& os) const override
    {
        TaskBenchmarkSettings::dump(os);
        os << "\n";
        os << "NumberOfMainPipeDamageMarkers: " << numberOfMainPipeDamageMarkers << "\n";
        os << "NumberOfBuoys: " << numberOfBuoys << "\n";
        os << buoysActions;
        os << "PipelinePipes:\n";
        for (const auto& pipe : pipelinePipes) {
            os << pipe;
        }
        os << "==========================================\n";
    }
};

struct SystemStatus {
    double timeout = 2.0; //s
    
    rclcpp::Time lastBridgeTime;
    rclcpp::Time lastPerceptionTime;
    rclcpp::Time lastKCLTime;

    rclcpp::Time lastStateSwitchTime;

    bool perceptionAlive = false;
    bool kclAlive = false;
    bool bridgeAlive = false;

    SystemStatus(rcl_clock_type_t clockType)
    {
        if (clockType == 1) {
            lastBridgeTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
            lastPerceptionTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
            lastKCLTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
            lastStateSwitchTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
        } else {
            lastBridgeTime = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
            lastPerceptionTime = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
            lastKCLTime = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
            lastStateSwitchTime = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
        }
    }

    bool IsAlive()
    {
        return perceptionAlive && kclAlive && bridgeAlive;
    }

    void UpdateStatus(rclcpp::Time now){
        perceptionAlive = lastPerceptionTime > (now - rclcpp::Duration::from_seconds(timeout));
        kclAlive = lastKCLTime > (now - rclcpp::Duration::from_seconds(timeout));
        bridgeAlive = lastBridgeTime > (now - rclcpp::Duration::from_seconds(timeout));

        #ifdef NO_BRIDGE
        bridgeAlive = true;
        #endif
        #ifdef NO_PERCEPTION
        perceptionAlive = true;
        #endif
        #ifdef NO_KCL
        kclAlive = true;
        #endif
    }
    
};

}

#endif // MISSION_CTRL_DATA_STRUCTS_HPP