#ifndef MISSION_CTRL_BASE_STATE_HPP
#define MISSION_CTRL_BASE_STATE_HPP

#include <fsm/fsm.h>
#include <libconfig.h++>
#include "ctrl_toolbox/HelperFunctions.h"
#include "mission_ctrl/mission_data_structs.hpp"

namespace mission {

namespace states {

    class StateBase : public fsm::BaseState {
        
    public:
        std::shared_ptr<mission::ControlData> ctrlData;
        std::shared_ptr<TaskBenchmarkSettings> taskData_;

        StateBase();
        virtual ~StateBase(void);
    };
}
}

#endif // MISSION_CTRL_BASE_STATE_HPP
