#!/bin/bash

###### DESCRIPTION ######
### Usage: differential inspection system
### Precaution: Run realsense-cam-X.sh First
######

### ARGUMENTS ###
LAUNCH_RVIZ=0 # 1 to launch rviz, otherwise don't
###

ROS_DISTRO=noetic
source /opt/ros/$ROS_DISTRO/setup.bash

cd /home/extraction-ws
source devel/setup.bash

# Launch differential
roslaunch diff-rs-pcl2 safety-ab.launch &

wait