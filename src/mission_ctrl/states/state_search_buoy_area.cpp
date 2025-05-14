#include "state_search_buoy_area.hpp"

namespace mission {

namespace states {

    StateSearchBuoyArea::StateSearchBuoyArea(){
    }

    StateSearchBuoyArea::~StateSearchBuoyArea() {
    }

    fsm::retval StateSearchBuoyArea::OnEntry(){           
        
        return fsm::ok;        
    }

    fsm::retval StateInit::Execute(){
        
    }

    fsm::retval StateInit::OnExit(){
        
        return fsm::ok;
    }
}
}
