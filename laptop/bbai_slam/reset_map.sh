#!/bin/bash
# Throw away the current map and restart gmapping from the robot's current spot
source ~/bbai_slam/env.sh
rosnode kill /slam_gmapping
# wait for the old node to exit, or the new one can be shut down by the name clash
for i in $(seq 1 20); do rosnode list 2>/dev/null | grep -q '^/slam_gmapping$' || break; sleep 0.5; done
nohup roslaunch ~/bbai_slam/gmapping_bbai.launch > ~/bbai_slam/gmapping.log 2>&1 &
echo "Map reset"
