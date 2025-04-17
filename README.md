# Mission Controller

## Overview

## Simulation Environment

Install [Stonefish](https://stonefish.readthedocs.io/en/latest/index.html) and [stonefish_utils](https://bitbucket.org/isme_robotics/stonefish_utils/src/main) together with their dependencies. 

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
