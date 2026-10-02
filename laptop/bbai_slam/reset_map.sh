#!/bin/bash
# Throw away the current map and restart gmapping from the robot's current spot
source ~/bbai_slam/env.sh
rosnode kill /slam_gmapping
sleep 2
nohup roslaunch ~/bbai_slam/gmapping_bbai.launch > ~/bbai_slam/gmapping.log 2>&1 &
echo "Map reset"
