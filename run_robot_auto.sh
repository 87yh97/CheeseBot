#!/bin/bash

#gnome-terminal -- catkin_make && roscore &
cd catkin_ws
catkin_make
cd ..
#$PID=$!
sleep 2
gnome-terminal -- roslaunch myrobot_control control.launch &
sleep 0.5
gnome-terminal -- roslaunch myrobot_simulator gazebo_testwalls.launch &
sleep 0.5
gnome-terminal -- roslaunch myrobot_description rviz.launch &
sleep 0.5
gnome-terminal -- roslaunch myrobot_teleop teleop_auto.launch

