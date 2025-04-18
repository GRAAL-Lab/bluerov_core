# Mission Controller

## Overview

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

## Dependencies

## Installation

## Author
[Samuele Depalo](samuele.depalo@edu.unige.it)