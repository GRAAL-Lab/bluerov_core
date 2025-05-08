# System Architecture

This document provides a full view of the architecture and associated finite state machines (FSMs) for the underwater robotics platform.

---

## 📦 High-Level Architecture

```mermaid
flowchart LR
    CONTROL[Control Station]
    MISSION[Mission Planner]
    KCL[KCL]
    BRIDGE[BlueROV Bridge]
    VEHICLE[BlueROV Vehicle]
    PERCEPTION[Perception]

    CONTROL <--> MISSION
    MISSION <--> KCL
    KCL <--> BRIDGE
    BRIDGE <--> VEHICLE
    VEHICLE --> PERCEPTION
    MISSION <--> PERCEPTION    
```

---

## 🔁 Mission Planner FSM

```mermaid
flowchart LR
    INIT(("INIT")) --> SEARCH_GATE(("SEARCH_GATE"))
    SEARCH_GATE --> TRAVERSE_GATE(("TRAVERSE_GATE"))
    TRAVERSE_GATE --> SEARCH_BUOY_AREA(("SEARCH_BUOY_AREA"))
    SEARCH_BUOY_AREA --> SOLVE_BUOY(("SOLVE_BUOY")) & REACH_PIPE_STRUCTS(("REACH_PIPE_STRUCTS"))
    SOLVE_BUOY -.-> SEARCH_BUOY_AREA
    REACH_PIPE_STRUCTS --> INSPECT_PIPES(("INSPECT_PIPES"))
    INSPECT_PIPES --> HOMING(("HOMING"))
    INIT -- orange --> SEARCH_MAIN_PIPE(("SEARCH_MAIN_PIPE"))
    SEARCH_MAIN_PIPE --> FOLLOW_MAIN_PIPE(("FOLLOW_MAIN_PIPE"))
    FOLLOW_MAIN_PIPE --> IDENTIFY_MANIPULATION_CONSOLE(("IDENTIFY_MANIPULATION_CONSOLE")) & REACH_PIPE_STRUCTS
    IDENTIFY_MANIPULATION_CONSOLE --> INTERVENTION(("INTERVENTION"))
    INTERVENTION --> HOMING
    SEARCH_BUOY_AREA -.-> FOLLOW_MAIN_PIPE
    INSPECT_PIPES -.-> IDENTIFY_MANIPULATION_CONSOLE
    linkStyle 0 stroke:#00C853,fill:none
    linkStyle 1 stroke:#00C853,fill:none
    linkStyle 2 stroke:#00C853,fill:none
    linkStyle 3 stroke:#00C853,fill:none
    linkStyle 4 stroke:#00C853,fill:none
    linkStyle 5 stroke:#00C853,fill:none
    linkStyle 6 stroke:#00C853,fill:none
    linkStyle 7 stroke:#00C853,fill:none
    linkStyle 8 stroke:#FFA500,fill:none
    linkStyle 9 stroke:#FFA500,fill:none
    linkStyle 10 stroke:#FFA500,fill:none
    linkStyle 11 stroke:#FFA500,fill:none
    linkStyle 12 stroke:#FFA500,fill:none
    linkStyle 13 stroke:#FFA500,fill:none

```

---

## 🧭 KCL FSM

```mermaid
stateDiagram-v2
  direction LR
  [*] --> IDLE
  IDLE --> PATH_FOLLOW:start_path_f
  IDLE --> HOLD:hold_pos
  IDLE --> SURFACE:surface
  IDLE --> WAYPOINT:go_to_wp
  PATH_FOLLOW --> HOLD:path_comp
  WAYPOINT --> HOLD:arrived
  SURFACE --> HOLD:depth=0

```

---

## 🔍 Perception FSM

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> STANDARD
    STANDARD --> IDLE
    STANDARD --> PIPES
    STANDARD --> BUOYS
    STANDARD --> MAIN_PIPE
    PIPES --> STANDARD
    BUOYS --> STANDARD
    MAIN_PIPE --> STANDARD
```

