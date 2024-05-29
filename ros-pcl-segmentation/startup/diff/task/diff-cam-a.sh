#!/bin/bash

###### DESCRIPTION ######
### Usage: turn on camera A
### Precaution: 
######

### ARGUMENTS ###
# Default values for parameters
camera="diff_cam_A"
serial_no="215222079777"
filters="spatial,pointcloud"
###


# Parse command line arguments for camera and serial_no, and use_filters
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--camera) camera="$2"; shift ;;
        -s|--serial-no) serial_no="$2"; shift ;;
        -f|--use-filters) use_filters=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Source the ROS setup script
ROS_DISTRO=noetic
source /opt/ros/$ROS_DISTRO/setup.bash

cd /home/realsense-ws
source devel/setup.bash

roslaunch realsense2_camera rs_camera.launch camera:=$camera serial_no:=$serial_no filters:=$filters

wait