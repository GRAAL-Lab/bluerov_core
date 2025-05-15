#ifndef MISSION_CTRL_DATA_STRUCTS_HPP
#define MISSION_CTRL_DATA_STRUCTS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>
#include <queue>

#include "ctrl_toolbox/HelperFunctions.h"
#include "mission_ctrl/mission_ctrl_defines.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission {

struct ControlData {
    ctb::LatLong inertialF_linearPosition;
    double depth;
    rml::EulerRPY bodyF_angularPosition;
};

enum BuoyAction {
    ClockWiseRotation = 0,
    CounterClockWiseRotation = 1,
    GoUp = 2,
    GoDown = 3
};
inline std::string BuoyActionToString(BuoyAction action)
{
    switch (action) {
    case ClockWiseRotation:
        return "ClockWiseRotation";
    case CounterClockWiseRotation:
        return "CounterClockWiseRotation";
    case GoUp:
        return "GoUp";
    case GoDown:
        return "GoDown";
    default:
        return "Unknown";
    }
}
enum BuoyColor {
    White = 0,
    Yellow = 1,
    Red = 2,
    Black = 3,
    Orange = 4
};
inline std::string BuoyColorToString(BuoyColor color)
{
    switch (color) {
    case White:
        return "White";
    case Yellow:
        return "Yellow";
    case Red:
        return "Red";
    case Black:
        return "Black";
    case Orange:
        return "Orange";
    default:
        return "Unknown";
    }
}

struct Buoy {
    ctb::LatLong position;
    double radius;
    BuoyColor color;
};

struct GateBuoy : public Buoy {
    GateBuoy(ctb::LatLong pos)
        : Buoy { pos, 0.1, BuoyColor::Orange }
    {
    }
};

struct DtcBuoy : public Buoy {
    DtcBuoy(ctb::LatLong pos, BuoyColor color)
        : Buoy { pos, 0.15, color }
    {
    }
};

struct Gate {
    GateBuoy buoy1;
    GateBuoy buoy2;
    double distanceTolerance = 0.5;
    double expectedDistance = 2.0;

    Gate(GateBuoy& b1, GateBuoy& b2)
        : buoy1(b1)
        , buoy2(b2)
    {
        Eigen::Vector3d distanceVector;
        ctb::LatLong2LocalNED(b1.position, 0, b2.position, distanceVector);
        if (distanceVector.norm() > expectedDistance + distanceTolerance || distanceVector.norm() < expectedDistance - distanceTolerance) {
            throw std::runtime_error("Gate buoys are too far apart!");
        }
    }
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
    double angleWithNorth;
    ctb::LatLong position;

    friend std::ostream& operator<<(std::ostream& os, const PipelinePipe& pipe)
    {
        os << "PipelinePipe {\n";
        os << "  number: " << pipe.number << "\n";
        os << "  angleWithNorth: " << pipe.angleWithNorth << "\n";
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

    TaskBenchmarkSettings() = default;

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

    bool GetPipelinePipesFromFile(libconfig::Config& confObj, std::vector<PipelinePipe>& pipelinePipes)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& pipelinePipesSetting = root["pipelinePipes"];
        for (int i = 0; i < pipelinePipesSetting.getLength(); ++i) {
            const libconfig::Setting& pipelinePipeSetting = pipelinePipesSetting[i];
            PipelinePipe pipe;
            if (!ctb::GetParam(pipelinePipeSetting, pipe.number, "number"))
                return false;
            if (!ctb::GetParam(pipelinePipeSetting, pipe.angleWithNorth, "orientation"))
                return false;
            if (!LatLongFromConfig(pipelinePipeSetting, pipe.position, "centroid"))
                return false;
            pipelinePipes.push_back(pipe);
        }
        return true;
    }

    bool GetBuoysActionsFromFile(libconfig::Config& confObj, std::map<BuoyAction, BuoyColor>& buoysActionsMap)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& buoysActions = root["buoysActions"];
        uint color;
        if (!ctb::GetParam(buoysActions, color, "clockWiseRotation"))
            return false;
        buoysActionsMap.emplace(ClockWiseRotation, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "counterClockWiseRotation"))
            return false;
        buoysActionsMap.emplace(CounterClockWiseRotation, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "goUp"))
            return false;
        buoysActionsMap.emplace(GoUp, static_cast<BuoyColor>(color));
        if (!ctb::GetParam(buoysActions, color, "goDown"))
            return false;
        buoysActionsMap.emplace(GoDown, static_cast<BuoyColor>(color));
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
    std::map<BuoyAction, BuoyColor> buoysActions;
    std::vector<PipelinePipe> pipelinePipes;

    Inspection()
    {
        taskType = taskBenchmarks::INSPECTION;
    }

    bool ConfigureFromFile(libconfig::Config& confObj) override
    {
        if (!TaskBenchmarkSettings::ConfigureFromFile(confObj))
            return false; // Call the base class code first!
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
        os << "BuoysActions:\n";
        for (auto const& a : buoysActions)
            os << "  " << BuoyActionToString(a.first)
               << " -> " << BuoyColorToString(a.second) << "\n";
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
        if (!ctb::GetParam(pipelinePipeSetting, damagedPipeOnPipeline.angleWithNorth, "angleWithNorth"))
            return false;
        if (!LatLongFromConfig(pipelinePipeSetting, damagedPipeOnPipeline.position, "centroid"))
            return false;

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
    std::map<BuoyAction, BuoyColor> buoysActions;
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
    void dump(std::ostream& os) const override
    {
        TaskBenchmarkSettings::dump(os);
        os << "\n";
        os << "NumberOfMainPipeDamageMarkers: " << numberOfMainPipeDamageMarkers << "\n";
        os << "NumberOfBuoys: " << numberOfBuoys << "\n";
        os << "BuoysActions:\n";
        for (const auto& action : buoysActions) {
            os << "  Action: " << BuoyActionToString(action.first)
               << " -> Color: " << BuoyColorToString(action.second) << "\n";
        }
        os << "PipelinePipes:\n";
        for (const auto& pipe : pipelinePipes) {
            os << pipe;
        }
        os << "==========================================\n";
    }
};

}

#endif // MISSION_CTRL_DATA_STRUCTS_HPP