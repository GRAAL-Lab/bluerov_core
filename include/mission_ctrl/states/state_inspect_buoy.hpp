#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateInspectBuoy : public StateBase {

    public:
        StateInspectBuoy();
        ~StateInspectBuoy() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;
    };

}
}