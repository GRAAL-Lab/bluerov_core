#include "mission_ctrl/states/state_inspect_pipes.hpp"

namespace mission {

namespace states {

    StateInspectPipes::StateInspectPipes(){
    }

    StateInspectPipes::~StateInspectPipes(){ 
    }

    fsm::retval StateInspectPipes::OnEntry(){

    }

    fsm::retval StateInspectPipes::Execute(){
       
        return fsm::ok;
    }

    fsm::retval StateInspectPipes::OnExit(){
       
        return fsm::ok;
    }

}
}
