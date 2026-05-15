#!/bin/bash

gnome-terminal -- bash -c "ros2 run ctrl_station terminal; exec bash"
gnome-terminal -- bash -c "ros2 run ctrl_station cmd_dispatcher; exec bash"
gnome-terminal -- bash -c "ros2 run ctrl_station mqtt_client; exec bash"
