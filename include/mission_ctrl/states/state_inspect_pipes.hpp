#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateInspectPipes : public StateBase {
    public:
        StateInspectPipes();
        ~StateInspectPipes() override;
        
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;
    };

}
}