#include "mission_ctrl/states/state_sleep.hpp"

namespace mission {

namespace states {

    StateSleep::StateSleep()
    {
    }

    StateSleep::~StateSleep() { }

    fsm::retval StateSleep::OnEntry()
    {
        sleepStartTime = std::chrono::steady_clock::now();
        return fsm::ok;
    }

    fsm::retval StateSleep::Execute()
    {
        double sleepTime = std::stod(taskData_->taskPhases.front().second);
        // auto past_time = now - std::chrono::seconds(seconds_input);
        auto elapsed = std::chrono::steady_clock::now() - sleepStartTime;
        if(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= sleepTime)
            return SetNextMissionState();
        return fsm::ok;
    }

    fsm::retval StateSleep::OnExit()
    {
        return fsm::ok;
    }
}
}
