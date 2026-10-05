# Robot command cheat sheet

Board `debian@192.168.3.120` · Laptop `jason@192.168.3.147` · Master `http://192.168.3.120:11311`

## Board (BeagleBone AI)

### ROS env (every new terminal; .bashrc points the master at the laptop)
| Command | What |
|---|---|
| `export ROS_MASTER_URI=http://192.168.3.120:11311 ROS_IP=192.168.3.120` | Use the board as master |
| `source ~/teleop_ws/devel/setup.bash` | Before teleop commands |
| `source ~/chase_ws/devel/setup.bash` | Before chase commands |

### Bringup and driving
| Command | What |
|---|---|
| `roslaunch bbai_base robot.launch` | Start first: master, lidar, motors, /odom, /imu, GPS |
| `roslaunch bbai_base robot.launch lidar:=false gps:=false` | Bringup without lidar / GPS |
| `roslaunch bbai_teleop teleop.launch` | PS4 teleop → /cmd_vel (after robot.launch) |
| `roslaunch bbai_chase chase.launch` | Dog chase, dry run (preview only) |
| `roslaunch bbai_chase chase.launch targets:="[dog, person]"` | Chase dog or person, dry run |
| `roslaunch bbai_chase chase.launch dry_run:=false` | Chase drives the motors (hold R2) |

### Calibration and tests
| Command | What |
|---|---|
| `rosrun bbai_base mag_calibrate.py [1.5]` | Compass cal: spins 2 turns on the floor, prints base.yaml lines |
| `nano ~/melodic-catkin-ws/src/bbai_base/config/base.yaml` | Paste the printed mag lines, restart robot.launch |
| `rosrun bbai_base motor_map_test.py` | Motor→wheel map (wheels lifted, robot.launch stopped) |
| `python3 ~/dog_detect.py 60` | Dog/person detection for 60 s (no ROS) |
| `python3 ~/oak_test.py 15` | OAK RGB + depth stream test |
| `python3 ~/oak_calcheck.py` | Read OAK calibration (read-only) |

### Bluetooth / PS4
| Command | What |
|---|---|
| `sudo systemctl restart bbai-bluetooth.service` | Restart Bluetooth (runs bt-up.sh) |
| `systemctl status bbai-bluetooth.service` | Check Bluetooth service |
| `~/teleop_ws/src/bbai_teleop/pair-ps4.sh` | Pair a controller (Share+PS first) |
| `ls /dev/input/js0` | Controller connected? |

### Checks and power
| Command | What |
|---|---|
| `rostopic list` / `rostopic hz /scan` | Topics up? lidar ~7 Hz |
| `rostopic echo /odom -n1` | Current odometry |
| `rostopic echo /gps/fix -n1` | GPS fix |
| `rostopic echo /chase/status` | Chase state |
| `nohup ~/bin/powerlog.py &` | Voltage logger (start after each boot) |
| `tail ~/power.log` | Last voltages |
| `journalctl -b -1 \| tail -50` | Why the last boot ended |

### Device tree
| Command | What |
|---|---|
| `sudo cp ~/src/BeagleBoard-DeviceTrees/src/arm/am5729-beagleboneai-roboticscape.dtb /boot/dtbs/4.14.108-ti-r136/ && sudo reboot` | Install cape dtb |
| `sudo cp ~/roboticscape.dtb.bak /boot/dtbs/4.14.108-ti-r136/am5729-beagleboneai-roboticscape.dtb && sudo reboot` | Roll back dtb |

## Laptop (Ubuntu, ROS Noetic)
| Command | What |
|---|---|
| `source ~/bbai_slam/env.sh` | Point ROS at the board |
| `~/bbai_slam/start_slam.sh` | gmapping SLAM + RViz (default) |
| `~/bbai_slam/start_slam.sh hector` | hector SLAM (lidar only) |
| `~/bbai_slam/reset_map.sh` | Wipe map, restart from current spot |
| `rosrun map_server map_saver -f ~/bbai_slam/<name>` | Save map (.pgm + .yaml) |
| `roslaunch ~/bbai_slam/gps_fusion.launch` | GPS + odom + IMU EKF → /odometry/global |
| `rostopic echo /odometry/global -n1` | Fused position |
| `rviz -d ~/bbai_slam/slam.rviz` | RViz only |

## Server VM (Proxmox, Ubuntu 20.04, ROS Noetic) · `jason@192.168.3.150`
| Command | What |
|---|---|
| `source ~/bbai_slam/env.sh` | Point ROS at the board |
| `~/bbai_slam/start_slam.sh` | gmapping SLAM, no RViz |
| `~/bbai_slam/explore_house.sh` | Map the house; Ctrl-C saves the map |
| `roslaunch foxglove_bridge foxglove_bridge.launch port:=8765` | Viewer link for Foxglove |
| `tmux new -s slam` / `tmux a -t slam` | Keep runs going after SSH closes / reattach |

## PC (Windows)
| Command | What |
|---|---|
| `ssh debian@192.168.3.120` | Board shell |
| `ssh jason@192.168.3.147` | Laptop shell |
| `ssh jason@192.168.3.150` | Server VM shell |
| Foxglove → `ws://192.168.3.150:8765` | View map, scan, plan |
| http://192.168.3.120:3000 | Cloud9 IDE on the board |
| PuTTY COM3, 115200 | Board serial console |
| `cd "%USERPROFILE%\OneDrive\Robot 2.0\Camera\calibration"` | OAK calibration folder |
| `C:\Users\jstum\venvs\oak-calib-v2\Scripts\python.exe calibrate.py -s 2.48 -ms 1.86 -brd OAK-D-LITE -ab 60 -dst C:\Users\jstum\oak-dataset -rlp cam_a=72 rgb=72` | OAK self-calibration (camera on PC; see Camera/CALIBRATION.md) |

## PS4 controller
| Input | What |
|---|---|
| Share + PS (hold) | Pairing mode (double flash) |
| PS | Reconnect |
| L1 (hold) | Enable driving (deadman); release = stop |
| L1 + left stick | Forward / back |
| L1 + right stick | Turn (1.5 rad/s) |
| L1 + R1 | Turbo |
| R2 (hold, L1 released) | Chase the target; release = stop |
