#!/bin/bash
source /opt/ros/humble/setup.bash
source ros_workspace/install/local_setup.bash

ros2 launch arm_robot_moveit_config demo.launch.py --noninteractive
ros2 run arm_package opencm_command3 --noninteractive # Does this need to be noninteractive?
