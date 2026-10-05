#!/bin/bash
# Board: start autonomous mode (board.launch). Run by bbai-robot.service at boot, or by hand.
# Extra roslaunch args come from BBAI_ARGS in /etc/default/bbai-robot, e.g. BBAI_ARGS="targets:=[dog,kid]"
source /opt/ros/melodic/setup.bash
for ws in melodic-catkin-ws teleop_ws chase_ws; do
  [ -f ~/$ws/devel/setup.bash ] && source ~/$ws/devel/setup.bash --extend
done
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=${ROS_IP:-192.168.3.120}
exec roslaunch bbai_autonomy board.launch $BBAI_ARGS "$@"
