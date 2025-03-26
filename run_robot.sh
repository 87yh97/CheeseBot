#!/bin/bash

gnome-terminal -- roslaunch myrobot_control control.launch &
sleep 2
gnome-terminal -- roslaunch myrobot_simulator gazebo_testwalls.launch &
sleep 2
gnome-terminal -- roslaunch myrobot_description rviz.launch

