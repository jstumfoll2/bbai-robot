#!/bin/bash
# Map the house automatically. The board must be running: roslaunch bbai_base robot.launch
# Ctrl-C stops exploring; the map is saved to ~/bbai_slam/maps/house_<time> on exit.
source ~/bbai_slam/env.sh
mkdir -p ~/bbai_slam/maps
# stop any SLAM already running so only one gmapping owns map->odom
rosnode kill /slam_gmapping /move_base /explore 2>/dev/null
sleep 2
roslaunch ~/bbai_slam/nav/explore.launch &
LAUNCH=$!
trap 'rosrun map_server map_saver -f ~/bbai_slam/maps/house_$(date +%Y%m%d_%H%M) ; kill $LAUNCH; wait $LAUNCH' INT TERM
sleep 5
rviz -d ~/bbai_slam/slam.rviz &
wait $LAUNCH
