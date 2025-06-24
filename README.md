# Mission Controller

## 🧭 System Overview

This system is composed of two primary ROS nodes:

- **`monitor_node`**: Responsible for checking the liveness of the following components:
  - `mission_ctrl`
  - `bridge`
  - `kcl`
  - `perception`

- **`mission_ctrl`**: This node sends high-level commands to `kcl` using a finite state machine (FSM) to execute mission tasks.

---

## 🚦 Benchmark Development Status

Development is tracked across **three Test Benchmarks (TBM)**. The status of each FSM state is listed below:

---

### 🧪 TBM 1

| FSM State             | Status                                                                 |
|----------------------|------------------------------------------------------------------------|
| `state_latlong`         | ✅ **Working** — _Obstacle avoidance not implemented_              |
| `state_search_gate`     | ⚠️ **Working** — _Untested; logic may be too simple_              |
| `state_cross_gate`      | ⚠️ **Working** — _Untested_                                       |
| `state_search_area`     | ⚠️ _Almost ready_ — _Still untested_                              |
| `state_inspect_buoy`    | ❌ _Incomplete_ — _Requires special action from `kcl`_             |
| `state_inspect_pipes`   | 🛠️ _Almost ready_ — _Awaiting perception & camera actuation_     |

---

### 🔧 TBM 2

| FSM State                             | Status                                                                 |
|--------------------------------------|------------------------------------------------------------------------|
| `state_main_pipe_following`            | ❌ _Not started_ — _Requires border perception & `kcl` pipe following_ |
| `state_search_for_manipulation_console`| 🛠️ _Almost ready_ — _Lacks distance keeping, perception, camera actuation, and likely a special `kcl` action_ |

---

### 🚧 TBM 3

_Note: All states listed, but no progress details provided yet._

- `state_main_pipe_following`
- `state_search_area`
- `state_inspect_buoy`
- `state_search_gate`
- `state_cross_gate`
- `state_inspect_pipes`
- `state_search_for_manipulation_console`

---

## 🗂️ Legend

- ✅ **Working** — Complete and tested
- ⚠️ **Working** — Untested or needs review
- 🛠️ **Almost Ready** — Pending minor components
- ❌ **Not Started / Incomplete** — Needs significant work

---

> ℹ️ *Technical terms (like node names and FSM states) have been preserved. Status indicators aim to help prioritize remaining work.*

   

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