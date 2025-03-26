#!/bin/bash

#gnome-terminal -- catkin_make && roscore &
#$PID=$!
sleep 1
gnome-terminal -- roslaunch myrobot_control control.launch &
sleep 2
gnome-terminal -- roslaunch myrobot_simulator gazebo_testwalls.launch &
sleep 2
gnome-terminal -- roslaunch myrobot_description rviz.launch &
sleep 2
gnome-terminal -- roslaunch myrobot_teleop teleop_keyboard.launch

