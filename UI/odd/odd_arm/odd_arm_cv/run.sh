#!/bin/bash

echo Launching nanoowl + cameras

source /opt/ros/humble/setup.bash
ros2 launch sense.py --noninteractive & jetson-containers run --name odd_arm_nanoowl --ipc=host --pid=host odd_arm/nanoowl:1.0.0 bash /opt/odd_arm_cv/nanoowl.sh & wait
