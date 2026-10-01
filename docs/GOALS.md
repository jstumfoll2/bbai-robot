# BeagleBone AI Robot — Goals & Status

_Last updated: 2026-10-01_

## Hardware

- **Controller:** BeagleBone AI (AM5729, 2× Cortex-A15, 4× PRU, EVE/DSP for TIDL), with the Robotics Cape
- **Drive:** small two-wheel platform, DC motors with A1230 Hall-effect wheel encoders
- **Sensors:** MPU-9250 IMU (on the cape), EM-506 GPS (SiRF, `/dev/ttyS2`, 4800 baud), RPLIDAR A1M8, OAK-D-Lite depth/AI camera, USB webcam
- **Available:** Raspberry Pi (model TBD), old laptop running Ubuntu (used for SLAM before)
- **Constraint:** the chassis is very small, so mounting extra compute (a Pi) is hard

## Goals

1. **Autonomous "chase the dog."** The robot recognizes Jason's dog with an on-board neural network and drives after it. This is the main goal.
   - It started with a USB webcam and the BeagleBone AI's TIDL classification demo, which identified basic objects.
   - The plan was to switch to the **OAK-D-Lite**: detection runs on the camera's own chip (Myriad X), and the camera's depth gives distance to the target.
2. **Map the house with SLAM.** Use the RPLIDAR A1 with wheel odometry and the IMU to build a 2D map. Later, use the map to navigate.
3. **Manual control mode.** Drive the robot remotely from an Android phone app or a Bluetooth gamepad (a PS4 controller). A DSM radio receiver was also considered (OrangeRX R110X).
4. **Sensor fusion in ROS.** Publish the IMU (`sensor_msgs/Imu`), wheel encoders (`nav_msgs/Odometry`, `tf`) and GPS (`sensor_msgs/NavSatFix`) as ROS topics so SLAM and navigation can use them.

## Where things stand (from memory plus the board survey on 2026-10-01)

| Area | Status |
|---|---|
| Board OS | Debian 10 Buster, kernel 4.14.108-ti-r136 (2020 TIDL image). On Wi-Fi at 192.168.3.120; Cloud9 on :3000 |
| Device tree | Custom `am5729-beagleboneai-roboticscape.dts` in `~/src/BeagleBoard-DeviceTrees`, built with `udt.sh` and booted through `uEnv.txt` |
| librobotcontrol | Local branch `v1.1_AIfixes`: 1 commit (2022-11-19) plus uncommitted PRU, encoder and servo fixes. Never pushed |
| Motors and encoders | Working with the basic librobotcontrol test programs (simple movement) |
| IMU | Reads fine. **Not yet published as a ROS topic.** This is where progress stalled |
| GPS | Tested. Poor tracking indoors |
| LIDAR | ROS Melodic built from source on the board, plus `rplidar_ros`. Basic SLAM mapping was done, with SLAM probably running on the Ubuntu laptop. How integrated it was is unknown |
| OAK-D-Lite | **Working on the board (2026-10-01).** depthai 2.19.1 (Python) runs RGB and depth at 15 fps over USB2. MobileNet-SSD `~/dog_detect.py` detects the dog at 93–99% confidence, about 10 detections/s, all on the camera's chip. **Self-calibrated and flashed 2026-10-01**: L/R epipolar error 0.36, baseline 7.43 cm, RGB lens fixed at 72. A backup JSON is in `Camera/`. The factory area is still empty; a Luxonis ticket is open. `depthai-ros` is not built yet |
| Buttons | Pause and Mode buttons don't work |
| Backups | Board work captured to `board-backup/` on 2026-10-01. Not yet in git |

## Open questions / decisions

- **Where should vision run?** The BeagleBone AI's old 32-bit OS is the blocker for depthai. Options:
  - (a) Upgrade the BeagleBone AI to a newer Debian image and retry depthai.
  - (b) Use the Raspberry Pi (preferably a 64-bit Pi 4 or 5) as the vision and ROS computer. The BeagleBone AI stays the real-time motor, encoder and IMU controller (PRUs), and the two link over Ethernet or USB.
  - (c) Put the OAK-D-Lite on the laptop for development only.
- **ROS version:** Melodic is end-of-life. Moving to ROS 2 (Humble or Jazzy) affects `rplidar_ros`, `depthai-ros` and `slam_toolbox`.
- **Mounting:** where the Pi or a larger battery could fit on the small chassis, or whether to move to a bigger platform (the DFRobot kit idea in Tasks.txt).

## Repos

- `jstumfoll2/bbai-robot`: robot-level code (device tree, scripts, ROS packages, docs). Currently empty
- `jstumfoll2/librobotcontrol`: proposed fork of `beagleboard/librobotcontrol` to hold the `v1.1_AIfixes` work
- `jstumfoll/beaglebot`: older repo (different GitHub account) that is already on the board
