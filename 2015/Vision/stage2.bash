#! /bin/bash

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
/opt/ros/indigo/bin/rosservice call /camera/start_capture
/opt/ros/indigo/bin/rosbag record -a -o /media/ExtraData/NAME.bag
