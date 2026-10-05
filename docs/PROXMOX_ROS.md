# Running the ROS SLAM stack on the Proxmox server

## Recommendation

Yes, move it. Run SLAM, navigation and exploration on an Ubuntu 20.04 VM on the Proxmox server, with no screen, and watch the map from the Windows PC in Foxglove. Keep the laptop as a backup viewer.

Why it's worth it:
- **It's always on.** The final plan (map the house, look for the dog, run on boot) needs an off-board brain that is up whenever the robot is. The laptop isn't.
- **It's wired.** The board is on Wi-Fi. With the laptop, every message crosses Wi-Fi twice (board to router to laptop). With the server on Ethernet it crosses once, which helps with the dropouts we saw during exploration.
- **More CPU.** gmapping, move_base and explore_lite all run there; the old laptop only has to draw RViz, if anything.

What stays the same: the BeagleBone stays the ROS master (`http://192.168.3.120:11311`) and runs `robot.launch`, so teleop and chase still work with the server off. The server runs exactly the same `bbai_slam` files the laptop runs.

Example addresses below: board `192.168.3.120`, VM `192.168.3.150`. Use any free address for the VM and change it everywhere you see `.150`.

## 1. Create the VM (Proxmox web UI, https://<proxmox-ip>:8006)

ROS 1 Noetic needs **Ubuntu 20.04**. Check your existing VM first:

```bash
lsb_release -ds
```

If it says 20.04, skip to step 2. If it's 22.04 or 24.04, Noetic won't install there; make a new VM for ROS (cleanest), or see "Docker instead" at the end.

1. Download `ubuntu-20.04.6-live-server-amd64.iso` to the Proxmox ISO storage: **local → ISO Images → Download from URL**, with
   `https://releases.ubuntu.com/20.04/ubuntu-20.04.6-live-server-amd64.iso`
2. **Create VM**:
   - General: name `ros`
   - OS: the 20.04 ISO
   - System: defaults, tick **Qemu Agent**
   - Disks: 32 GB, VirtIO SCSI
   - CPU: 4 cores, **Type: host**
   - Memory: 4096 MB (8192 if you have it)
   - Network: **Bridge vmbr0**, Model **VirtIO**, **untick Firewall**
3. Start it, open **Console**, install Ubuntu Server. Tick **Install OpenSSH server**. Username `jason`.

`vmbr0` is a bridge, so the VM gets its own address on your home network, just like a physical PC. That's what ROS 1 needs: the board must be able to open connections *to* the VM, not only the other way around. Don't use NAT.

## 2. Give the VM a fixed address

Easiest: in your router, add a DHCP reservation for the VM's MAC address (Proxmox: VM → Hardware → Network Device) to `192.168.3.150`, then reboot the VM.

Or set it in the VM with netplan. Check the interface name with `ip -br a` (usually `ens18`), then:

```bash
sudo nano /etc/netplan/00-installer-config.yaml
```
```yaml
network:
  version: 2
  ethernets:
    ens18:
      addresses: [192.168.3.150/24]
      routes: [{to: default, via: 192.168.3.1}]
      nameservers: {addresses: [192.168.3.1, 1.1.1.1]}
```
```bash
sudo netplan apply
```

(Replace `192.168.3.1` with your router's address if it's different.)

From the Windows PC you can now `ssh jason@192.168.3.150`. Do the rest over SSH.

## 3. Basics: guest agent, clock, firewall

```bash
sudo apt update && sudo apt install -y qemu-guest-agent chrony git
sudo systemctl enable --now qemu-guest-agent
sudo ufw status            # should say inactive; if not: sudo ufw allow from 192.168.3.0/24
```

**Clocks matter.** ROS compares timestamps between the board and the VM. If they disagree by more than a fraction of a second, you get the same "extrapolation into the past" errors as the Wi-Fi dropout. Compare them:

```bash
ssh debian@192.168.3.120 date +%s.%N; date +%s.%N
```

The two numbers should be within about 0.1 s (plus the second or so SSH takes). If the board is off by more, check `timedatectl` on the board; it should say `System clock synchronized: yes`.

## 4. Install ROS Noetic and the packages SLAM uses

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-ros-base ros-noetic-gmapping ros-noetic-hector-slam \
  ros-noetic-navigation ros-noetic-explore-lite ros-noetic-robot-localization \
  ros-noetic-map-server ros-noetic-foxglove-bridge
```

`ros-base` has no RViz or desktop, which is what you want on a server. Everything the laptop built in `~/noetic-catkin-ws` comes from apt here, so no workspace is needed.

## 5. Copy the SLAM files and saved maps

```bash
git clone https://github.com/jstumfoll2/bbai-robot.git ~/bbai-robot
cp -r ~/bbai-robot/laptop/bbai_slam ~/bbai_slam
scp -r jason@192.168.3.147:~/bbai_slam/maps ~/bbai_slam/     # maps made on the laptop
echo 'source ~/bbai_slam/env.sh' >> ~/.bashrc
source ~/bbai_slam/env.sh
```

`env.sh` now works out its own address (`ROS_IP`) from whichever machine it runs on, so the same file works on the laptop and the VM. Check it:

```bash
echo $ROS_IP          # should print 192.168.3.150
```

## 6. Test the link both ways

Start the robot as usual on the board (`roslaunch bbai_base robot.launch`). Then on the VM:

```bash
rostopic list                 # should list /scan, /odom, /imu ...
rostopic hz /scan             # about 7 Hz
```

That proves VM → board. Now board → VM. On the VM:

```bash
roslaunch ~/bbai_slam/gmapping_bbai.launch
```

and on the board, in a second SSH window:

```bash
rostopic echo /map -n1 | head -5     # map data published by the VM
rosnode ping -c3 /slam_gmapping
```

If `rostopic list` works but `/map` never arrives on the board, the board can't reach the VM. Check the VM's firewall, that `ROS_IP` on the VM is `192.168.3.150`, and that the VM network is `vmbr0` and not NAT.

## 7. Run mapping from the VM

Same scripts as the laptop. Over SSH there's no screen, so they skip RViz and keep running.

```bash
~/bbai_slam/start_slam.sh            # gmapping, Ctrl-C to stop
~/bbai_slam/explore_house.sh         # autonomous exploration, Ctrl-C saves the map
~/bbai_slam/reset_map.sh
```

To keep a run going after you close SSH, start it inside `tmux` (`tmux new -s slam`, detach with Ctrl-B then D, come back with `tmux a -t slam`). Install with `sudo apt install -y tmux` if it's missing.

Only one machine should run SLAM at a time. Two gmappings both publish map → odom and the robot jumps between them.

## 8. Watch it: Foxglove on the Windows PC

Foxglove is a free viewer that runs on Windows and talks to ROS over one websocket. Nothing ROS-related needs to be on the PC.

On the VM, in its own terminal or tmux window:

```bash
roslaunch foxglove_bridge foxglove_bridge.launch port:=8765
```

On the PC:
1. Install Foxglove from https://foxglove.dev/download and sign in (free account).
2. **Open connection → Foxglove WebSocket**, URL `ws://192.168.3.150:8765`.
3. Add a **3D** panel. In its settings set **Display frame** to `map`, and turn on `/map`, `/scan`, the move_base plan topics, and `/explore/frontiers`. Add an **Image** panel for the camera when that's publishing.
4. Save the layout so you don't have to set it up again.

To drive a goal by hand, use the 3D panel's **Publish → Pose** tool on `/move_base_simple/goal`.

**Backup: RViz on the laptop.** The laptop can still be just a viewer while the VM does the work: `source ~/bbai_slam/env.sh && rviz -d ~/bbai_slam/slam.rviz`. Don't start SLAM there.

## Later: start on boot

Once you're happy with it, the VM's SLAM and the Foxglove bridge can start as systemd services so the server is always ready when the robot powers on. That's part of the run-on-boot plan for the robot, so it'll be set up together with that.

## Docker instead (if you'd rather keep your 22.04/24.04 VM)

Noetic runs fine in a container with host networking, which gives it the VM's address directly:

```bash
sudo apt install -y docker.io
sudo docker run -it --name ros --network host -v ~/bbai_slam:/root/bbai_slam ros:noetic-ros-base bash
```

Inside, run the `apt install` line from step 4 (without `sudo`), then continue from step 5. `--network host` is required; the default Docker network is NAT and the board couldn't reach it.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `rostopic list` hangs | Board not running `robot.launch`, or VM can't reach `192.168.3.120` |
| Topics listed but `echo` shows nothing | Board can't reach the VM back: firewall, `ROS_IP`, or NAT instead of `vmbr0` |
| "Extrapolation into the past" | Clocks differ between board and VM, or the board's Wi-Fi dropped |
| Robot jumps between two positions | SLAM running on both the laptop and the VM |
| Foxglove won't connect | `foxglove_bridge` not running, or port 8765 blocked |
