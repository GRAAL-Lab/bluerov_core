#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateSleep : public StateBase {

        bool doneInit = false;

    public:
        StateSleep();
        ~StateSleep() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        std::chrono::time_point<std::chrono::steady_clock> sleepStartTime;
    };

}
}