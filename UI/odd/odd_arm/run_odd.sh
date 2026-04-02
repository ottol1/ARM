#!/bin/bash
source /opt/ros/humble/setup.bash
source ros_workspace/install/local_setup.bash

ros2 launch odd_package all_launch.py --noninteractive 
