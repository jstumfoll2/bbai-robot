#!/bin/bash
# Start SLAM on the laptop: ./start_slam.sh [gmapping|hector|lidar_only], default gmapping
# The board must already be running: roslaunch bbai_base robot.launch
source ~/bbai_slam/env.sh
roslaunch ~/bbai_slam/${1:-gmapping}_bbai.launch &
sleep 5
rviz -d ~/bbai_slam/slam.rviz
