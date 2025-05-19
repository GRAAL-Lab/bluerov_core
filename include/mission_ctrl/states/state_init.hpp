#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateInit : public StateBase {

    public:
        StateInit();
        ~StateInit() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        bool doneInit;
    };

}
}