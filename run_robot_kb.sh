#!/bin/bash

#gnome-terminal -- catkin_make && roscore &
gnome-terminal -- catkin_make &
#$PID=$!
sleep 1
gnome-terminal -- roslaunch myrobot_control control.launch &
sleep 0.5
gnome-terminal -- roslaunch myrobot_simulator gazebo_testwalls.launch &
sleep 0.5
gnome-terminal -- roslaunch myrobot_teleop teleop_keyboard.launch

