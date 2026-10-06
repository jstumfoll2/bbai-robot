# bbai_autonomy: search the house, chase on sight, start on boot

The robot's default behaviour. It maps the house with the lidar, searches it for the dog
(or the kids), chases whatever it finds, and goes back to searching when the target is gone.

```
first boot, no saved map          every boot after that
  EXPLORE  gmapping + explore_lite    PATROL  map_server + AMCL on house.yaml
     |  frontier to frontier             |  search points 1.5 m apart, nearest first,
     |  until none are left              |  turn to look around at each, then repeat
     v                                   |
  save maps/house_<date>.yaml  ------->  +--> forever
            (any time a target is seen: CHASE, then resume)
```

## Who runs what

| Where | Launch | Nodes |
|---|---|---|
| Board (BeagleBone AI, Melodic) | `board.launch` (systemd `bbai-robot.service`) | `robot.launch`, PS4 teleop, `chase` in auto mode, `cmd_mux` |
| Host (Proxmox VM or laptop, Noetic) | `host.launch` via `host_autonomy.sh` (systemd `bbai-host@<user>`) | gmapping + explore_lite **or** map_server + AMCL, `move_base`, `mission` |

The host is any machine that sources `~/bbai_slam/env.sh`; nothing in here names the laptop or
the VM. If the host is off or loses Wi-Fi, `nav/cmd_vel` stops and the motors stop after 0.5 s;
the board still teleops and still chases.

## Who drives

`cmd_mux` on the board owns `/cmd_vel` in autonomous mode:

1. **L1 held**: PS4 teleop drives; the mux sends nothing.
2. **Chase**: while `chase` tracks a target it drives (`chase/cmd_vel`).
3. **Navigation**: otherwise `move_base` drives (`nav/cmd_vel`).

**Options** on the controller turns autonomy off and on (`/autonomy/enabled`). Off means the robot
sits still unless you drive it with L1.

## Targets

`dog`, `kid`, `adult`, `person` (everyone), or any MobileNet-SSD label. Change them live:
`rostopic pub -1 /chase/set_targets std_msgs/String "dog,kid"`.

MobileNet-SSD has no child class. `kid` is a person whose head is under `kid_max_height` (1.35 m)
above the floor, worked out from the depth and where the top of the box is in the image, so the
feet don't need to be in view. If the head is above the top of the frame and the frame top there
is already above 1.35 m, it's an adult; otherwise it can't tell and doesn't chase, except to keep
following a kid it was already tracking. Set `camera_height` and `camera_pitch_deg` in
`chase.launch` to the real mount, then watch `rostopic echo /chase/status` with a kid and an
adult in view: it prints `kid(1.12m tall)`.

## Saved map, not a fresh one each time

The house doesn't change, so it maps once and localizes after that:

- AMCL on a saved map is far lighter and steadier than gmapping, and it can't smear the map
  while chasing the dog around a room.
- Search points come from the finished map, so every room gets visited each round instead of
  the robot stopping once the frontiers run out.
- Mapping is slow with the A1. Doing it once, then checking the map, beats redoing it every boot.

AMCL needs a starting guess. The mission saves the robot's pose every 5 s to
`maps/last_pose.yaml` and uses it at the next start, so turning the robot off where it stopped
just works. If you carry it somewhere else, put it at the spot where mapping started (the map's
origin) and send `home`, or send `global` to let AMCL search the whole map.

Moved the furniture? `rostopic pub -1 /autonomy/command std_msgs/String remap` forgets the map
and maps again. Old maps stay in `~/bbai_slam/maps/house_<date>.*`; `house.yaml` is a link to
the current one.

## Install

Board (the chase package changes too; `git clone https://github.com/jstumfoll2/bbai-robot.git ~/bbai-robot` first if it is not there):

```bash
cd ~/bbai-robot && git pull
cp ~/bbai-robot/ros/bbai_chase/scripts/chase_node.py ~/chase_ws/src/bbai_chase/scripts/
cp ~/bbai-robot/ros/bbai_chase/launch/chase.launch ~/chase_ws/src/bbai_chase/launch/
ln -s ~/bbai-robot/ros/bbai_autonomy ~/chase_ws/src/bbai_autonomy
cd ~/chase_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Host (VM or laptop):

```bash
cd ~/bbai-robot && git pull
mkdir -p ~/autonomy_ws/src && ln -s ~/bbai-robot/ros/bbai_autonomy ~/autonomy_ws/src/
cd ~/autonomy_ws && source /opt/ros/noetic/setup.bash && catkin_make
```

## Try it by hand first

1. Board: `~/chase_ws/src/bbai_autonomy/systemd/robot_start.sh autostart:=false`
2. Host: `~/autonomy_ws/src/bbai_autonomy/systemd/host_autonomy.sh`
3. Press **Options** to start. Press it again (or hold L1) to stop.
4. Watch `rostopic echo /autonomy/state` and `/autonomy/source`, and the map in Foxglove or RViz.

## Turn on at boot

```bash
# board
sudo cp ~/chase_ws/src/bbai_autonomy/systemd/bbai-robot.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now bbai-robot.service
# host
sudo cp ~/autonomy_ws/src/bbai_autonomy/systemd/bbai-host@.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now bbai-host@$USER
```

Board options go in `/etc/default/bbai-robot`, for example
`BBAI_ARGS="targets:=[dog,kid] autostart:=false"`. With `autostart:=false` it boots idle and
waits for **Options**, which is the safer default until it has run a few times.

## Topics

| Topic | Type | What |
|---|---|---|
| `/autonomy/state` | String | `EXPLORE ...`, `PATROL round 2 point 5/14`, `MAPPED house_...` |
| `/autonomy/source` | String | who is driving: `off`, `idle`, `teleop`, `chase`, `nav` |
| `/autonomy/enabled` | Bool | autonomy on or off |
| `/autonomy/enable` | Bool | turn it on or off from a terminal |
| `/autonomy/command` | String | `remap`, `save` (finish mapping now), `home`, `global` |
| `/chase/active` | Bool | a target is tracked |
| `/chase/set_targets` / `/chase/targets` | String | change / read the targets |
