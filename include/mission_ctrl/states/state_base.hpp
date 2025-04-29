#ifndef MISSION_CTRL_BASE_STATE_HPP
#define MISSION_CTRL_BASE_STATE_HPP

#include <fsm/fsm.h>
#include <libconfig.h++>
#include "ctrl_toolbox/HelperFunctions.h"
#include "mission_ctrl/mission_data_structs.hpp"

namespace mission {

namespace states {

    class StateBase : public fsm::BaseState {
    protected:
        // std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;
        // std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask_;

        //double minHeadingError_, maxHeadingError_;

    public:
        std::shared_ptr<mission::ControlData> ctrlData;
        std::shared_ptr<TaskBenchmarkSettings> taskData_;
        // std::shared_ptr<tpik::ActionManager> actionManager;
        // std::shared_ptr<rml::RobotModel> robotModel;
        // std::unordered_map<std::string, TasksInfo> tasksMap;

        StateBase();
        virtual ~StateBase(void);

        //void CheckRadioController();
        //virtual bool ConfigureStateFromFile(libconfig::Config& confObj) = 0;
    };
}
}

#endif // MISSION_CTRL_BASE_STATE_HPP
