#! /bin/bash

cd ~/catkin_ws
source /opt/ros/indigo/setup.bash
source devel/setup.bash
catkin_make
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch car_serial_comms training.launch 
