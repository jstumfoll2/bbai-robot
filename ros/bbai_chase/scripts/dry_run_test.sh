#!/bin/bash
# Dry-run smoke test for bbai_chase: start chase.launch (dry_run, dog+person) for ~40 s,
# then print status, preview twists and who publishes /cmd_vel. Needs robot.launch's master up.
source /opt/ros/melodic/setup.bash
source ~/chase_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.3.120:11311 ROS_IP=192.168.3.120
timeout -k 10 -s INT 60 roslaunch bbai_chase chase.launch dry_run:=true targets:="[dog, person]" > /tmp/chase_dry.log 2>&1 &
sleep 20
echo "== /chase/status"; timeout 15 rostopic echo -n 6 /chase/status | grep data
echo "== /chase/cmd_vel_preview (linear.x / angular.z)"; timeout 15 rostopic echo -n 3 /chase/cmd_vel_preview | grep -A1 -e 'linear' -e 'angular' | grep -e ' x:' -e ' z:'
echo "== /cmd_vel publishers"; rostopic info /cmd_vel | sed -n '/Publishers/,/Subscribers/p'
wait
echo "== chase log tail"; grep -v '^\[1944' /tmp/chase_dry.log | grep -e TRACK -e SEARCH -e ERROR -e OAK | tail -n 12
