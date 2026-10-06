# Point this shell at the ROS master on the BeagleBone AI (laptop or server VM)
source /opt/ros/noetic/setup.bash
[ -f ~/noetic-catkin-ws/devel/setup.bash ] && source ~/noetic-catkin-ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.3.120:11311
# this machine's address on the route to the board, so the same file works on any machine
export ROS_IP=$(ip route get 192.168.3.120 | sed -n 's/.* src \([0-9.]*\).*/\1/p')
