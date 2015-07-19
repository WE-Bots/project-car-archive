#! /bin/bash

cd ~/catkin_ws
source /opt/ros/indigo/setup.bash
source devel/setup.bash
catkin_make
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch src/iarrcMlVision/launch/run_control.launch
