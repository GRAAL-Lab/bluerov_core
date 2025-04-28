#ifndef MISSION_CTRL_DATA_STRUCTS_HPP
#define MISSION_CTRL_DATA_STRUCTS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox/HelperFunctions.h"

// #include "MISSION_msgs/msg/task_status.hpp"


namespace rami {

    enum BuoyAction {
        ClockWiseRotation = 0,
        CounterClockWiseRotation = 1,
        GoUp = 2,
        GoDown = 3
    };

    enum BuoyColor {
        White = 0,
        Yellow = 1,
        Red = 2,
        Black = 3
    };

    struct PipelinePipe {
        uint number;
        double angleWithNorth;
        ctb::LatLong position;
    };

    struct MissionConfiguration {
        std::vector<ctb::LatLong> pipelineStructures;

        bool enableBuoysArea;
        ctb::LatLong buoysAreaCentroid;
        std::vector<double> buoysAreaSize;
        double buoysAreaOrientation;

        uint pipelineStructureId;
        uint numberOfMainPipeDamageMarkers;
        uint numberOfBuoys;
        std::map<BuoyAction, BuoyColor> buoysActions;
        std::vector<PipelinePipe> pipelinePipes;
        PipelinePipe damagedPipeOnPipeline;
    };

struct ControlData {
    // ctb::LatLong inertialF_linearPosition;
    // rml::EulerRPY bodyF_angularPosition;
    // Eigen::Vector3d bodyF_linearVelocity;
    // Eigen::Vector3d bodyF_angularVelocity;
    // Eigen::Vector2d inertialF_waterCurrent;
    // bool radioControllerEnabled;

    // ControlData() : radioControllerEnabled(false) {}
};

struct TasksInfo {

    // std::shared_ptr<tpik::Task> task;
    // rclcpp::Publisher<MISSION_msgs::msg::TaskStatus>::SharedPtr taskPub;
};

enum class ControlMode : int {
    ThrusterMapping,
    ClassicPIDControl,
    ComputedTorque
};

struct KCLConfiguration {

    // bool goToHoldAfterMove;
    // double posAcceptanceRadius;
    // double controlLoopRate;
    // Eigen::VectorXd saturationMin, saturationMax;

    // KCLConfiguration()
    //     : goToHoldAfterMove(false)
    // {
    // }

    // bool ConfigureFromFile(libconfig::Config& confObj)
    // {

    //     if (!ctb::GetParam(confObj, goToHoldAfterMove, "goToHoldAfterMove"))
    //         return false;
    //     if (!ctb::GetParam(confObj, controlLoopRate, "controlLoopRate"))
    //         return false;
    //     if (!ctb::GetParam(confObj, posAcceptanceRadius, "posAcceptanceRadius"))
    //         return false;
    //     if (!ctb::GetParamVector(confObj, saturationMax, "saturationMax"))
    //         return false;
    //     if (!ctb::GetParamVector(confObj, saturationMin, "saturationMin"))
    //         return false;

    //     return true;
    // }

    // friend std::ostream& operator<<(std::ostream& os, KCLConfiguration const& a)
    // {
    //     return os << "======= KCL CONF =======\n"
    //               << "ControlLoopRate: " << a.controlLoopRate << "\n"
    //               << "PosAcceptanceRadius: " << a.posAcceptanceRadius << "\n"
    //               << "GoToHoldAfterMove: " << a.goToHoldAfterMove << "\n"
    //               << "SaturationMin: " << a.saturationMin.transpose() << "\n"
    //               << "SaturationMax: " << a.saturationMax.transpose() << "\n"
    //               << "===============================\n";
    // }
};

}

#endif //  MISSION_CTRL_DATA_STRUCTS_HPP
