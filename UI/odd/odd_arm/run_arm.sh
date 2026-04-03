#!/bin/bash
source /opt/ros/humble/setup.bash
source ros_workspace/install/setup.bash

ros2 launch arm_robot_moveit_config demo.launch.py --noninteractive
sudo chmod 666 /dev/ttyACM0 # Make sure this is the OpenCM
ros2 run arm_package opencm_command3 --noninteractive # Does this need to be noninteractive?
