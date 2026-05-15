# Mission Controller

## System Overview

This system is composed of two primary ROS nodes:

- **`monitor_node`**: Responsible for checking the liveness of the following components: (`mission_ctrl`, `bridge`, `kcl`, `perception`)

- **`mission_ctrl`**: This node sends high-level commands to `kcl` using a finite state machine (FSM) to execute mission tasks.

---

## Benchmark Development Status

The status of each FSM state is listed below:

---

### TBM 1

| FSM State             | Status                                                                 |
|----------------------|------------------------------------------------------------------------|
| `state_latlong`         | ✅ **Working** — _Obstacle avoidance not implemented_              |
| `state_search_gate`     | ⚠️ **Working** — _Untested; logic may be too simple_              |
| `state_cross_gate`      | ⚠️ **Working** — _Untested_                                       |
| `state_search_area`     | ⚠️ _Almost ready_ — _Untested_                              |
| `state_inspect_buoy`    | ❌ _Incomplete_ — _Requires special action from `kcl`_             |
| `state_inspect_pipes`   | 🛠️ _Almost ready_ — _Awaiting perception & camera actuation_     |

---

### TBM 2

| FSM State                             | Status                                                                 |
|--------------------------------------|------------------------------------------------------------------------|
| `state_main_pipe_following`            | ❌ _Not started_ — _Requires border perception & `kcl` pipe following_ |
| `state_search_for_manipulation_console`| 🛠️ _Almost ready_ — _Lacks distance keeping, perception, camera actuation, and likely a special `kcl` action_ |

---

### TBM 3

_Note: All states listed, read above for progress detail._

- `state_main_pipe_following`
- `state_search_area`
- `state_inspect_buoy`
- `state_search_gate`
- `state_cross_gate`
- `state_inspect_pipes`
- `state_search_for_manipulation_console`

---

## Task-independent Behavior

These behaviors run independently of mission states to ensure the **safety** and **reliability** of the vehicle.

| Behavior                          | Status       | Description |
|----------------------------------|--------------|-------------|
| **System liveness check**        | ✅ **Implemented** | `monitor_node` observes key components. If a component fails to send messages within a set time, the system is marked non-operational and `kcl` is commanded to go `idle`. |
| **Safety borders enforcement**   | ✅ **Implemented** | The vehicle must stay within predefined mission boundaries. Exiting the area triggers an `idle` command. A grace period can be configured. |
| **KCL execution watchdog**       | ✅ **Implemented** | If `kcl` stops sending feedback before task completion, the same command is re-sent until confirmed complete by the FSM. |
| **State timeout mechanism**      | 🛠️ _Partially implemented_ | Each FSM state can define a timeout. If exceeded, the mission is halted. |
| **Vehicle inactivity detection** | ❌ _Not implemented_ | Detects when the vehicle stops while it should be in motion. |
| **KCL state mismatch detection** | ❌ _Not implemented_ | Identifies when `kcl` is in a different state than the one it was commanded to enter. |
| **Idle switch monitoring**       | ❌ _Not implemented_ | Monitors unexpected transitions out of `idle` mode. |

---

## Simulation Environment

Install [Stonefish](https://stonefish.readthedocs.io/en/latest/index.html) and [stonefish_utils](https://bitbucket.org/isme_robotics/stonefish_utils/src/main) together with their dependencies. 

> ⚠️ **Reminder**: Make sure to install the specific version of Stonefish used in this project with the following steps:
>
> ```bash
> git clone "https://github.com/patrykcieslak/stonefish.git"
> cd stonefish
> git reset --hard 64ddc7c31b2b800610bd6c55dbb1a378422c0873
> mkdir build
> cd build
> cmake ..
> make -jX     # Replace X with the number of cores to speed up compilation
> sudo make install
> ```

Run the example:

```shell
ros2 launch stonefish_utils stonefish.py scenario:=rami_scenarios/example.scn
```

Move the robot with:
```shell 
ros2 launch stonefish_utils bluerovTeleop.py <-record>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Author
[Samuele Depalo](samuele.depalo@edu.unige.it)