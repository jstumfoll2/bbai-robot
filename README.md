# bbai-robot

Robot-level code for a small four-wheel skid-steer robot built on a BeagleBone AI with the Robotics
Cape: RPLIDAR A1 SLAM, wheel odometry and IMU in ROS, PS4 teleop over Bluetooth, and OAK-D-Lite dog
detection for the "chase the dog" goal. Goals and current state are in [docs/GOALS.md](docs/GOALS.md)
and [docs/HANDOFF.md](docs/HANDOFF.md).

| Folder | What's in it |
|---|---|
| `devicetree/` | Robotics Cape device tree for the BB-AI (encoder pull-ups, UART6 Bluetooth, PRU motor pin) |
| `pru/` | Notes for the PRU2_0 `gpio_rpmsg` firmware (source is in the librobotcontrol submodule) |
| `librobotcontrol/` | Submodule: [jstumfoll2/librobotcontrol](https://github.com/jstumfoll2/librobotcontrol) `v1.1_AIfixes` |
| `ros/` | Catkin packages for the board (ROS Melodic): `bbai_base`, `bbai_teleop`, `bbai_chase` |
| `laptop/` | SLAM launch files and scripts for the Ubuntu/Noetic laptop |
| `camera/` | OAK-D-Lite scripts for the board, and PC calibration tools plus the calibration backup |
| `gps/` | libgps patch for the EM-506 on `/dev/ttyS2` |
| `system/` | Board boot config, module blacklist, helper scripts |
| `docs/` | Goals, handoff notes, camera calibration guide |

## ROS packages

On the board, each package lives in a catkin workspace (as of 2026-10-01):

| Package | Workspace on the board | Launch |
|---|---|---|
| `bbai_base` | `~/melodic-catkin-ws` | `roslaunch bbai_base robot.launch`: master, rplidar, `/odom`, `/imu/data_raw`, `/cmd_vel` motor control |
| `bbai_teleop` | `~/teleop_ws` | `roslaunch bbai_teleop teleop.launch`: PS4 controller to `/cmd_vel` |
| `bbai_chase` | `~/chase_ws` | `roslaunch bbai_chase chase.launch`: follows the dog with the OAK-D-Lite |

Build Python 3 workspaces with `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`. Start `robot.launch`
first so it owns the master.

## Cloning

```bash
git clone --recurse-submodules https://github.com/jstumfoll2/bbai-robot.git
```
