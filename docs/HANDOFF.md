# Handoff notes: state as of 2026-10-01

Read this together with `GOALS.md`. It is for anyone (or any new Claude thread) picking up the work.

## Access

- **BeagleBone AI:** Wi-Fi `192.168.3.120`, user `debian`. Cloud9 IDE at http://192.168.3.120:3000.
  - Serial debug console on PC **COM3**, 115200 8N1. Only one program can hold COM3 at a time, so close PuTTY before Claude uses it.
  - sudo needs the password, so Claude asks Jason to run sudo commands.
  - SSH key login works from the PC (`jstum@StarLab` ed25519 key, added 2026-10-01): `ssh debian@192.168.3.120`. To add the laptop, run `ssh-copy-id debian@192.168.3.120` from it.
- **PC:** `C:\Users\jstum\oak-venv` is Python 3.11 with depthai **3.10.0** (v3), for calibration tools. gh CLI is logged in as jstumfoll2.
- **Ubuntu laptop:** ROS tools for lidar SLAM. Address and ROS version not recorded yet.

## Board software

- Debian 10 Buster, kernel 4.14.108-ti-r136, Python 3.7.
  - apt now points to `archive.debian.org`, with the security line commented out.
- ROS Melodic, built from source in `~/ros_catkin_ws` and installed to `/opt/ros/melodic`.
  - `~/melodic-catkin-ws/src/rplidar_ros`
  - `~/dai_ws/src/depthai-ros` (not building yet)
- RPLIDAR A1 is wired to a Robotics Cape UART, not USB. Which `/dev/ttyS*` it uses is not confirmed yet: check the rplidar_ros launch files.

## Workstreams

1. **OAK-D-Lite / dog detection.** Works on the board with depthai 2.19.1.
   - `~/oak_test.py` streams RGB and depth (15 fps).
   - `~/oak_calcheck.py` reads the calibration (read-only).
   - `~/dog_detect.py` runs MobileNet-SSD with `~/models/mobilenet-ssd_openvino_2021.4_6shave.blob`. Dog seen at 93–99% confidence.
   - **The EEPROM calibration is erased.** Luxonis asked for a v3 dump, done with `Camera/calib_dump_v3.py` on the PC (camera plugged into the PC). Fallback is self-calibration: `Camera/CALIBRATION.md`, with `Camera/charuco_11x8.pdf` as the target.
   - Next: write the "chase" control loop (bearing → steering, z → speed) using librobotcontrol motors.
2. **Lidar SLAM.** The A1 publishes from the board with rplidar_ros (Melodic); SLAM runs on the Ubuntu laptop.
   - Needs: the ROS_MASTER_URI / ROS_IP network setup between the board and the laptop, the UART port, and odometry from the encoders and IMU.
3. **Repos / backup.**
   - Board snapshot: `board-backup/board-backup-2026-10-01.tgz`.
   - https://github.com/jstumfoll2/librobotcontrol, branch `v1.1_AIfixes`. It has the 2022 commit, plus the recovered uncommitted PRU and encoder fixes.
   - `jstumfoll2/bbai-robot` is still empty. Plan: device tree, scripts, these docs, and the test scripts above, with librobotcontrol as a submodule.
4. **Manual control.** PS4 controller over Bluetooth, or a phone app. Not started.
