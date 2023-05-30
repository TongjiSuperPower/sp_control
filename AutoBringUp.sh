#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

sudo ip link set can3 up type can bitrate 1000000
sudo ip link set can4 up type can bitrate 1000000
sudo ip link set can5 up type can bitrate 1000000
sudo chmod 777 /dev/ttyUSB0

sleep 1s
gnome-terminal --tab "engineer-control" -- bash -c "roslaunch sp_hw load_engineer.launch;exec bash;"

sleep 10s
{
gnome-terminal --tab "manipulator-control" -- bash -c "source ~/catkin_ws/devel/setup.bash;rosrun manipulator_control manipulator_control;exec bash;"
}&
