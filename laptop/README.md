# SLAM laptop scripts

Ubuntu 20.04 laptop with ROS Noetic (`~/noetic-catkin-ws` has hector_slam and gmapping). Copy
`bbai_slam/` to `~/bbai_slam`. The board runs the ROS master.

1. On the board: `roslaunch bbai_base robot.launch`
2. On the laptop: `~/bbai_slam/start_slam.sh [gmapping|hector|lidar_only]` (gmapping is the default, since it uses `/odom`)
3. `reset_map.sh` restarts gmapping from the robot's current spot.
4. Save a map: `source env.sh && rosrun map_server map_saver -f <name>`

Edit the board address in `env.sh` if it changes; `ROS_IP` is detected. The same files run on the Proxmox server VM: see `docs/PROXMOX_ROS.md`.
