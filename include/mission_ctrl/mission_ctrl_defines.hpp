#ifndef MISSION_CTRL_DEFINES_HPP
#define MISSION_CTRL_DEFINES_HPP

#include <string>

namespace mission {

namespace taskBenchmarks {
    const std::string INSPECTION = "INSPECTION";
    const std::string INTERVENTION = "INTERVENTION";
    const std::string INSPECTION_AND_INTERVENTION = "INSPECTION & INTERVENTION";
}

namespace states {
    namespace ID {
        const std::string init = "Init";
        const std::string moveToWp = "MoveToWp";
        const std::string moveToDepth = "moveToDepth";
        const std::string searchForObject = "SearchForObject"; // gate, main pipe, manipulation console
        const std::string crossGate = "CrossGate";
        const std::string searchBuoyArea = "SearchBuoyArea";
        const std::string inspectBuoy = "InspectBuoy";
        const std::string inspectPipes = "InspectPipes";
        const std::string followMainPipe = "FollowMainPipe";
        const std::string interventionOnConsole = "InterventionOnConsole";
        const std::string updateLocalization = "UpdateLocalization";
        const std::string homing = "Homing";
        const std::string halt = "Halt";
    }
}

namespace opis {
    const std::string uavWaypoint = "UavWaypoint";
    const std::string gate = "Gate";
    const std::string mainPipe = "MainPipe";
    const std::string manipulationConsole = "ManipulationConsole";
    const std::string pipelineStructure = "PipelineStructure";
}

}
#endif // MISSION_CTRL_DEFINES_HPP