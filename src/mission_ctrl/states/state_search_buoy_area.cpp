#include "mission_ctrl/states/state_search_buoy_area.hpp"

namespace mission {

namespace states {

    StateSearchBuoyArea::StateSearchBuoyArea(){
    }

    StateSearchBuoyArea::~StateSearchBuoyArea() {
    }

    fsm::retval StateSearchBuoyArea::OnEntry(){           
        
        return fsm::ok;        
    }

    fsm::retval StateSearchBuoyArea::Execute(){
        
        return fsm::ok;       
    }

    fsm::retval StateSearchBuoyArea::OnExit(){
        
        return fsm::ok;
    }
}
}
