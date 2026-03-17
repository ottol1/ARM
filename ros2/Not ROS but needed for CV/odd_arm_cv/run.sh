#!/bin/bash
sudo echo Launching NanoOwl and cameras
(sudo bash sense.sh & jetson-containers run --ipc=host --pid=host odd_arm/nanoowl:1.0.0 bash /opt/odd_arm_cv/nanoowl.sh & wait)
