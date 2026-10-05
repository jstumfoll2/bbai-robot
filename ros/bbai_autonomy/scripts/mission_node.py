#!/usr/bin/env python3
"""Mission manager for autonomous mode. Runs on the off-board host next to move_base.

mode "explore" (no saved map yet)
  gmapping builds the map and explore_lite drives to frontiers. When explore_lite has
  sent no new goal for ~idle_timeout seconds, the house is mapped: the map is saved to
  <map_dir>/house_<time>.{pgm,yaml}, house.yaml is pointed at it, and this node exits.
  host_autonomy.sh then restarts everything in patrol mode on the saved map.

mode "patrol" (saved map, AMCL localization)
  Sets the AMCL start pose, spreads search waypoints over the free space of the map,
  and drives to them one after another (nearest first), looking around at each one.
  When every waypoint has been visited it starts the next round. Forever.

Chasing is not done here: chase_node (arm_mode auto) drives on its own whenever it sees
a target, and the board's cmd_mux gives it the wheels over move_base. In patrol mode
this node cancels the current goal while /chase/active is true, then resumes once the
target has been gone for resume_delay seconds. In explore mode move_base just keeps
planning in the background and continues when the chase ends.

Topics
  sub  chase/active        std_msgs/Bool
  sub  autonomy/enabled    std_msgs/Bool (from cmd_mux; waits while disabled)
  sub  autonomy/command    std_msgs/String: "remap" (forget the map and explore again),
                           "save" (explore mode: finish now), "home" (patrol: robot is at
                           the map origin), "global" (patrol: AMCL global localization)
  sub  move_base/goal, move_base/status (explore mode: is explore_lite still working?)
  pub  autonomy/state      std_msgs/String (latched)
  pub  initialpose         geometry_msgs/PoseWithCovarianceStamped
"""
import math
import os
import subprocess
import time
from collections import deque

import numpy as np
import rospy
import actionlib
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Mission(object):
    def __init__(self):
        gp = rospy.get_param
        self.mode = gp("~mode", "explore")
        self.map_dir = os.path.expanduser(gp("~map_dir", "~/bbai_slam/maps"))
        self.map_name = gp("~map_name", "house")
        # explore
        self.min_explore_time = gp("~min_explore_time", 120.0)
        self.idle_timeout = gp("~idle_timeout", 60.0)
        self.partial_save_period = gp("~partial_save_period", 120.0)
        # patrol
        self.spacing = gp("~waypoint_spacing", 1.5)     # m between search points
        self.clearance = gp("~clearance", 0.30)         # m from walls and unknown space
        self.look_around = gp("~look_around", True)     # turn in place at each point
        self.goal_timeout = gp("~goal_timeout", 90.0)
        self.resume_delay = gp("~resume_delay", 3.0)
        self.use_last_pose = gp("~use_last_pose", True)
        self.pose_save_period = gp("~pose_save_period", 5.0)

        self.chase_stamp = -1e9
        self.enabled = True
        self.last_goal_t = time.monotonic()
        self.goal_active = False
        self.command = None

        self.state_pub = rospy.Publisher("autonomy/state", String, queue_size=1, latch=True)
        self.init_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        rospy.Subscriber("chase/active", Bool, self.on_chase, queue_size=1)
        rospy.Subscriber("autonomy/enabled", Bool, self.on_enabled, queue_size=1)
        rospy.Subscriber("autonomy/command", String, self.on_command, queue_size=1)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        os.makedirs(self.map_dir, exist_ok=True)

    # ---------------------------------------------------------------- inputs
    def on_chase(self, msg):
        # chase_stamp = last time a target was tracked (chase publishes at ~15 Hz)
        if msg.data:
            self.chase_stamp = time.monotonic()

    def chasing(self):
        """True while chasing, and for resume_delay after the target is lost.
        A silent chase node counts as not chasing."""
        return time.monotonic() - self.chase_stamp < self.resume_delay

    def on_enabled(self, msg):
        self.enabled = msg.data

    def on_command(self, msg):
        self.command = msg.data.strip().lower()
        rospy.loginfo("mission: command '%s'", self.command)

    def on_goal(self, _):
        self.last_goal_t = time.monotonic()

    def on_status(self, msg):
        if any(s.status in (GoalStatus.ACTIVE, GoalStatus.PENDING) for s in msg.status_list):
            self.goal_active = True
            self.last_goal_t = time.monotonic()
        else:
            self.goal_active = False

    def state(self, s):
        self.state_pub.publish(s)
        rospy.loginfo_throttle(5.0, "mission: " + s)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(0.5))
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # ---------------------------------------------------------------- maps
    def map_path(self, name):
        return os.path.join(self.map_dir, name)

    def save_map(self, name):
        """map_saver -f <map_dir>/<name>; True on success."""
        try:
            r = subprocess.run(["rosrun", "map_server", "map_saver", "-f", self.map_path(name)],
                               timeout=30, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return r.returncode == 0 and os.path.exists(self.map_path(name) + ".yaml")
        except Exception as e:
            rospy.logwarn("mission: map save failed: %s", e)
            return False

    def point_current_map(self, name):
        link = self.map_path(self.map_name + ".yaml")
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(name + ".yaml", link)

    # ---------------------------------------------------------------- explore
    def run_explore(self):
        rospy.Subscriber("move_base/goal", MoveBaseActionGoal, self.on_goal, queue_size=5)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.on_status, queue_size=1)
        start = time.monotonic()
        last_partial = start
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            now = time.monotonic()
            if self.command == "save":
                break
            self.command = None  # the other commands are for patrol mode
            if not self.enabled:
                self.last_goal_t = now  # paused: not finished
                self.state("EXPLORE paused")
            elif self.chasing():
                self.last_goal_t = now
                self.state("EXPLORE chasing")
            else:
                idle = now - self.last_goal_t
                self.state("EXPLORE %.0fs, goal %s, idle %.0fs" % (
                    now - start, "active" if self.goal_active else "none", idle))
                if now - start > self.min_explore_time and not self.goal_active and idle > self.idle_timeout:
                    break
            if now - last_partial > self.partial_save_period:
                self.save_map(self.map_name + "_partial")
                last_partial = now
            r.sleep()
        if rospy.is_shutdown():
            return
        name = "%s_%s" % (self.map_name, time.strftime("%Y%m%d_%H%M"))
        self.state("SAVING map " + name)
        if self.save_map(name):
            self.point_current_map(name)
            self.save_pose()  # patrol starts AMCL from here, not from the map origin
            rospy.loginfo("mission: house mapped, saved %s; switching to patrol", self.map_path(name))
            self.state("MAPPED " + name)
        else:
            rospy.logerr("mission: could not save the map; exploring again")
        rospy.signal_shutdown("explore finished")

    # ---------------------------------------------------------------- patrol
    def set_initial_pose(self, x, y, yaw, sigma_xy=0.3, sigma_yaw=0.3):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x, p.pose.pose.position.y = x, y
        q = yaw_quat(yaw)
        o = p.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = sigma_xy ** 2
        cov[35] = sigma_yaw ** 2
        p.pose.covariance = cov
        self.init_pub.publish(p)
        rospy.loginfo("mission: initial pose %.2f %.2f %.0fdeg", x, y, math.degrees(yaw))

    def start_pose(self):
        f = self.map_path("last_pose.yaml")
        if self.use_last_pose and os.path.exists(f):
            try:
                d = yaml.safe_load(open(f))
                if d.get("map") == os.path.realpath(self.map_path(self.map_name + ".yaml")):
                    return d["x"], d["y"], d["yaw"]
            except Exception as e:
                rospy.logwarn("mission: bad last_pose.yaml (%s)", e)
        return 0.0, 0.0, 0.0  # where mapping started

    def save_pose(self):
        p = self.robot_pose()
        if p is None:
            return
        f = self.map_path("last_pose.yaml")
        with open(f + ".tmp", "w") as fh:
            yaml.safe_dump({"x": p[0], "y": p[1], "yaw": p[2],
                            "map": os.path.realpath(self.map_path(self.map_name + ".yaml"))}, fh)
        os.replace(f + ".tmp", f)

    def waypoints(self, grid, robot):
        """Search points on a spacing x spacing grid over reachable free space."""
        info = grid.info
        res, w, h = info.resolution, info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.array(grid.data, dtype=np.int16).reshape(h, w)
        safe = (data >= 0) & (data < 25)
        k = max(1, int(round(self.clearance / res)))
        for _ in range(k):  # erode by one cell per pass, 8-connected
            e = safe.copy()
            e[1:, :] &= safe[:-1, :]
            e[:-1, :] &= safe[1:, :]
            e[:, 1:] &= safe[:, :-1]
            e[:, :-1] &= safe[:, 1:]
            e[1:, 1:] &= safe[:-1, :-1]
            e[:-1, :-1] &= safe[1:, 1:]
            e[1:, :-1] &= safe[:-1, 1:]
            e[:-1, 1:] &= safe[1:, :-1]
            e[0, :] = e[-1, :] = False
            e[:, 0] = e[:, -1] = False
            safe = e
        cells = np.argwhere(safe)
        if len(cells) == 0:
            return []
        # flood fill from the safe cell nearest the robot: only reachable points
        rx, ry = (robot[0] - ox) / res, (robot[1] - oy) / res
        sy, sx = cells[np.argmin((cells[:, 0] - ry) ** 2 + (cells[:, 1] - rx) ** 2)]
        reach = np.zeros_like(safe)
        reach[sy, sx] = True
        q = deque([(sy, sx)])
        while q:
            cy, cx = q.popleft()
            for ny, nx in ((cy + 1, cx), (cy - 1, cx), (cy, cx + 1), (cy, cx - 1)):
                if 0 <= ny < h and 0 <= nx < w and safe[ny, nx] and not reach[ny, nx]:
                    reach[ny, nx] = True
                    q.append((ny, nx))
        step = max(1, int(round(self.spacing / res)))
        pts = []
        for by in range(0, h, step):
            for bx in range(0, w, step):
                block = np.argwhere(reach[by:by + step, bx:bx + step])
                if len(block) == 0:
                    continue
                c = (step - 1) / 2.0
                j = np.argmin((block[:, 0] - c) ** 2 + (block[:, 1] - c) ** 2)
                cy, cx = block[j][0] + by, block[j][1] + bx
                pts.append((float(ox + (cx + 0.5) * res), float(oy + (cy + 0.5) * res)))
        return pts

    def order(self, pts, start):
        """Nearest-neighbour tour from start."""
        left, tour, cur = list(pts), [], start
        while left:
            i = min(range(len(left)), key=lambda j: math.hypot(left[j][0] - cur[0], left[j][1] - cur[1]))
            cur = left.pop(i)
            tour.append(cur)
        return tour

    def goto(self, x, y, yaw, label):
        """Drive to a pose. Pauses for chases and while disabled. True if reached."""
        while not rospy.is_shutdown():
            while (self.chasing() or not self.enabled) and not rospy.is_shutdown():
                self.state("PATROL %s (%s)" % (label, "chasing" if self.enabled else "paused"))
                rospy.sleep(0.5)
            if self.command:
                return False
            g = MoveBaseGoal()
            g.target_pose.header.frame_id = "map"
            g.target_pose.header.stamp = rospy.Time.now()
            g.target_pose.pose.position.x, g.target_pose.pose.position.y = x, y
            o = g.target_pose.pose.orientation
            o.x, o.y, o.z, o.w = yaw_quat(yaw)
            self.mb.send_goal(g)
            t0 = time.monotonic()
            interrupted = False
            while not rospy.is_shutdown():
                if self.mb.wait_for_result(rospy.Duration(0.25)):
                    break
                if self.chasing() or not self.enabled or self.command:
                    self.mb.cancel_goal()
                    interrupted = True
                    break
                if time.monotonic() - t0 > self.goal_timeout:
                    self.mb.cancel_goal()
                    return False
                self.state("PATROL %s" % label)
            if interrupted:
                continue  # resume the same goal once the chase or pause is over
            return self.mb.get_state() == GoalStatus.SUCCEEDED
        return False

    def handle_command(self):
        c, self.command = self.command, None
        if c == "remap":
            link = self.map_path(self.map_name + ".yaml")
            if os.path.lexists(link):
                os.remove(link)
            rospy.loginfo("mission: map forgotten, restarting in explore mode")
            rospy.signal_shutdown("remap")
        elif c == "home":
            self.set_initial_pose(0.0, 0.0, 0.0)
        elif c == "global":
            try:
                rospy.ServiceProxy("global_localization", Empty)()
                rospy.loginfo("mission: AMCL global localization; the first patrol legs will settle it")
            except Exception as e:
                rospy.logwarn("mission: global_localization failed: %s", e)

    def run_patrol(self):
        self.state("PATROL waiting for the map")
        grid = rospy.wait_for_message("map", OccupancyGrid)
        rospy.sleep(2.0)  # let amcl subscribe
        self.set_initial_pose(*self.start_pose())
        rospy.Timer(rospy.Duration(self.pose_save_period), lambda _: self.save_pose())
        rnd = 0
        while not rospy.is_shutdown():
            if self.command:
                self.handle_command()
                continue
            pose = self.robot_pose() or (0.0, 0.0, 0.0)
            pts = self.waypoints(grid, pose)
            if not pts:
                self.state("PATROL no reachable free space; check localization")
                rospy.sleep(5.0)
                continue
            rnd += 1
            tour = self.order(pts, pose[:2])
            if rnd % 2 == 0:
                tour.reverse()  # alternate direction so no room is always last
            rospy.loginfo("mission: patrol round %d, %d search points", rnd, len(tour))
            for i, (x, y) in enumerate(tour):
                if rospy.is_shutdown() or self.command:
                    break
                label = "round %d point %d/%d" % (rnd, i + 1, len(tour))
                p = self.robot_pose() or pose
                yaw = math.atan2(y - p[1], x - p[0])
                if not self.goto(x, y, yaw, label):
                    rospy.loginfo("mission: skipped %s", label)
                    continue
                if self.look_around:
                    for k in (1, 2):
                        if not self.goto(x, y, yaw + k * 2.0 * math.pi / 3.0, label + " look"):
                            break

    def run(self):
        self.state("STARTING %s" % self.mode)
        rospy.loginfo("mission: waiting for move_base")
        while not self.mb.wait_for_server(rospy.Duration(5.0)):
            if rospy.is_shutdown():
                return
            self.state("STARTING %s, waiting for move_base" % self.mode)
        if self.mode == "patrol":
            self.run_patrol()
        else:
            self.run_explore()


if __name__ == "__main__":
    rospy.init_node("mission")
    Mission().run()
