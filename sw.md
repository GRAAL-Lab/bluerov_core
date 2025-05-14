# System Architecture

This document provides a full view of the architecture and associated finite state machines (FSMs) for the underwater robotics platform.

---
```mermaid
flowchart LR
 subgraph MISSION_PLANNER["Mission Planner FSM"]
        SEARCH_GATE("SEARCH_GATE")
        INIT("INIT")
        TRAVERSE_GATE("TRAVERSE_GATE")
        SEARCH_BUOY_AREA("SEARCH_BUOY_AREA")
        SOLVE_BUOY("SOLVE_BUOY")
        REACH_PIPE_STRUCTS("REACH_PIPE_STRUCTS")
        INSPECT_PIPES("INSPECT_PIPES")
        HOMING("HOMING")
        SEARCH_MAIN_PIPE("SEARCH_MAIN_PIPE")
        FOLLOW_MAIN_PIPE("FOLLOW_MAIN_PIPE")
        IDENTIFY_MANIPULATION_CONSOLE("IDENTIFY_MANIPULATION_CONSOLE")
        INTERVENTION("INTERVENTION")
  end
 subgraph PERCEPTION_SYSTEM["Perception FSM"]
        P_STANDARD["STANDARD"]
        P_IDLE["IDLE"]
        P_PIPES["PIPES"]
        P_BUOYS["BUOYS"]
        P_MAIN_PIPE["MAIN_PIPE"]
  end
 subgraph KCL_MODULE["KCL FSM"]
        K_IDLE["IDLE"]
        K_PATH_FOLLOW["PATH_FOLLOW"]
        K_HOLD["HOLD"]
        K_SURFACE["SURFACE"]
        K_WAYPOINT["WAYPOINT"]
  end
    INIT --> SEARCH_GATE & SEARCH_MAIN_PIPE & SEARCH_MAIN_PIPE
    SEARCH_GATE --> TRAVERSE_GATE
    TRAVERSE_GATE --> REACH_PIPE_STRUCTS & SEARCH_BUOY_AREA
    SEARCH_BUOY_AREA --> REACH_PIPE_STRUCTS & TRAVERSE_GATE
    SOLVE_BUOY <--> SEARCH_BUOY_AREA & SEARCH_BUOY_AREA
    REACH_PIPE_STRUCTS --> INSPECT_PIPES & INSPECT_PIPES & IDENTIFY_MANIPULATION_CONSOLE
    INSPECT_PIPES --> HOMING & IDENTIFY_MANIPULATION_CONSOLE
    SEARCH_MAIN_PIPE --> FOLLOW_MAIN_PIPE & FOLLOW_MAIN_PIPE
    FOLLOW_MAIN_PIPE --> REACH_PIPE_STRUCTS & SEARCH_BUOY_AREA
    IDENTIFY_MANIPULATION_CONSOLE --> INTERVENTION & INTERVENTION
    INTERVENTION --> HOMING & HOMING
    P_IDLE --> P_STANDARD
    P_STANDARD --> P_IDLE & P_PIPES & P_BUOYS & P_MAIN_PIPE
    P_PIPES --> P_STANDARD
    P_BUOYS --> P_STANDARD
    P_MAIN_PIPE --> P_STANDARD
    K_IDLE <--> K_PATH_FOLLOW & K_WAYPOINT & K_SURFACE & K_HOLD
    K_PATH_FOLLOW <--> K_HOLD
    K_WAYPOINT <--> K_HOLD
    K_SURFACE <--> K_HOLD
    CONTROL_STATION["Control Station"] --Ros Service--> MISSION_PLANNER
    MISSION_PLANNER <--Ros Service--> KCL_MODULE & PERCEPTION_SYSTEM
    KCL_MODULE <--> BRIDGE["BlueROV Bridge"]
    BRIDGE <--Mavlink--> VEHICLE["Vehicle"]
    VEHICLE --H.264 Camera Stream--> PERCEPTION_SYSTEM
    linkStyle 0 stroke:#00C853,fill:none
    linkStyle 2 stroke:#FF6D00,fill:none
    linkStyle 3 stroke:#00C853,fill:none
    linkStyle 4 stroke:#000000,fill:none
    linkStyle 5 stroke:#00C853,fill:none
    linkStyle 6 stroke:#00C853,fill:none
    linkStyle 7 stroke:#000000,fill:none
    linkStyle 9 stroke:#00C853,fill:none
    linkStyle 10 stroke:#00C853,fill:none
    linkStyle 11 stroke:#000000,fill:none
    linkStyle 12 stroke:#FF6D00,fill:none
    linkStyle 13 stroke:#00C853,fill:none
    linkStyle 14 stroke:#000000,fill:none
    linkStyle 15 stroke:#000000,fill:none
    linkStyle 16 stroke:#FF6D00,fill:none
    linkStyle 17 stroke:#FF6D00,fill:none
    linkStyle 20 stroke:#FF6D00,fill:none
    linkStyle 22 stroke:#FF6D00,fill:none
```

---

### Notes
Legend for arrows color in Mission planner FSM:
- green = TBM1
- orange = TBM2
- black = TBM3