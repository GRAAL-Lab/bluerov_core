string pipeline_pipe_code       #empty, A or B. (depending on pp orientations given by the bros)
bool pipe_in_fov                #spotted a pipe!
LatLong point_on_pipe           #try to keep this constant unless it is not on the pipe anymore
float64 vertical_distance       #robot-pipe distance on z-axis
float64 pipe_direction          #NED 
bool move_toward_structure      #this will be false at start, true once reached the end of the pipe (opposite to structure)

bool found_red_marker
Position red_marker_position
bool found_number
Position number_position
uint8 number # number spotted on current pipe
