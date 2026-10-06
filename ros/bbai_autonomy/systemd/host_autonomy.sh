#!/bin/bash
# Off-board host (laptop or Proxmox VM): SLAM, move_base and the mission for autonomous mode.
# No saved map -> explore (map the house, save it). Saved map -> patrol (localize, search).
# Waits for the robot, and restarts everything when the robot reboots (new roscore).
# Run by bbai-host@<user>.service, or by hand. Ctrl-C to stop.
source ~/bbai_slam/env.sh
source ~/autonomy_ws/devel/setup.bash --extend
MAPS=${BBAI_MAP_DIR:-$HOME/bbai_slam/maps}
mkdir -p "$MAPS"
LAUNCH=
trap '[ -n "$LAUNCH" ] && kill -INT $LAUNCH; wait; exit 0' INT TERM
while true; do
  until RUN_ID=$(rosparam get /run_id 2>/dev/null); do
    echo "waiting for the robot at $ROS_MASTER_URI"; sleep 5
  done
  if [ -e "$MAPS/house.yaml" ]; then MODE=patrol; else MODE=explore; fi
  echo "robot up (run_id $RUN_ID), starting $MODE"
  roslaunch bbai_autonomy host.launch mode:=$MODE map_dir:="$MAPS" &
  LAUNCH=$!
  # stay while the launch runs and the robot's roscore is the same one
  while kill -0 $LAUNCH 2>/dev/null; do
    sleep 5
    NOW=$(rosparam get /run_id 2>/dev/null)
    if [ "$NOW" != "$RUN_ID" ]; then
      echo "robot restarted or unreachable, restarting"
      kill -INT $LAUNCH; wait $LAUNCH
      break
    fi
  done
  wait $LAUNCH 2>/dev/null
  LAUNCH=
  sleep 3
done
