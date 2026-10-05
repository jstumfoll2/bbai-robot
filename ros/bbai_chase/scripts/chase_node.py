#!/usr/bin/env python3
"""Chase-the-dog node for the BeagleBone AI robot.

Runs MobileNet-SSD spatial detection on the OAK-D-Lite (all inference on the
camera's Myriad X) and steers toward the best target, driving up to a follow distance.
arm_mode "button" (default): it drives only while the chase button is held on the PS4
controller. arm_mode "auto": it drives whenever a target is tracked; bbai_autonomy's
cmd_mux remaps its output and decides whether chasing is allowed.

Topics
  sub  /joy                   sensor_msgs/Joy (from bbai_teleop)
  sub  /imu/data_raw          sensor_msgs/Imu, bias-corrected gyro z (from bbai_base)
  sub  /chase/set_targets     std_msgs/String, e.g. "dog" or "dog,kid": changes targets live
  pub  /chase/target          geometry_msgs/PointStamped, camera frame: x right, z forward (m)
  pub  /chase/status          std_msgs/String, e.g. "TRACK dog 92% 1.40m -8.2deg"
  pub  /chase/active          std_msgs/Bool, true while a target is tracked well enough to chase
  pub  /chase/targets         std_msgs/String (latched), the current target list
  pub  /cmd_vel               geometry_msgs/Twist, only while armed (dry_run:=false)
  pub  /chase/cmd_vel_preview geometry_msgs/Twist, what it would send (always)

Targets
  Any MobileNet-SSD label (dog, person, cat, ...) plus "kid" and "adult". The network has
  no child class, so a person is sorted by how high the top of their head is above the
  floor: camera_height + depth x tan(angle of the box top above the camera axis, plus
  camera_pitch). Under kid_max_height (1.35 m) is a kid, over it an adult. The feet don't
  need to be in view, which matters with the camera this low. When the box touches the
  top of the frame the head is out of view: if even the frame top is above
  kid_max_height there it is an adult, otherwise "person" (can't tell). Once a kid is
  being tracked, "can't tell" views keep the track (a kid up close fills the frame).
  "person" matches everyone.

Safety
  - In button mode it moves only while the chase button is held AND the teleop
    deadman (L1) is not. In auto mode it moves while a target is tracked and L1 is
    not held. Teleop always wins.
  - One zero Twist on release, on target loss (> lost_timeout), on joy timeout, on exit.
  - No reverse. Stops inside stop_distance. Speed capped by max_linear, turning by max_turn_cmd.
  - dry_run (default true) never publishes /cmd_vel. It only fills the preview topic.

Turning
  The bearing loop asks for a yaw rate, but a skid-steer robot needs several times
  more wheel torque to start a spin than the plain cmd_vel mapping gives, so on the
  floor small turn commands stall. A yaw-rate loop on the gyro raises the angular.z
  sent to bbai_base until the robot actually turns at the requested rate.
  Detections arrive ~0.15 s late at 10 Hz, so the target is tracked as a fixed
  direction in the room (bearing at capture minus the gyro heading then) and turned
  back into a bearing with the current gyro heading. Without that, the robot keeps
  turning on stale bearings and the stiction lurch overshoots back and forth.

Field of view
  The 300x300 detector preview is a centre crop of the 16:9 frame (keep aspect ratio),
  so the network sees about 39 deg, not the full ~65 deg. The fallback bearing (no
  depth in the box) uses the preview's field of view, computed from the calibration.
"""
import collections
import math
import threading
import time

# Import order matters: on this board, loading numpy's C extension (pulled in by
# rospy/genpy) *after* depthai's native library deadlocks in dlopen. Keep numpy and
# rospy above depthai.
import numpy  # noqa: F401
import rospy
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, String

import depthai as dai

LABELS = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
          "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
          "sheep", "sofa", "train", "tvmonitor"]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class YawRateLoop(object):
    """PI loop on measured yaw rate around a feed-forward of the requested rate.

    out = w_ref + kp*(w_ref - w_gyro) + I, with I += ki*dt*(w_ref - w_gyro).
    The integrator learns how much extra command the floor needs to scrub the tyres.
    It resets when no turn is requested or the turn direction flips, and stops
    integrating while the output is saturated (anti-windup). Without a recent gyro
    reading it falls back to open loop with a minimum spin command.
    """

    def __init__(self, kp, ki, max_out, min_spin, gyro_timeout):
        self.kp, self.ki, self.max_out = kp, ki, max_out
        self.min_spin, self.gyro_timeout = min_spin, gyro_timeout
        self.integral = 0.0
        self.last_t = None

    def reset(self):
        self.integral = 0.0
        self.last_t = None

    def update(self, w_ref, w_gyro, gyro_age, now):
        if w_ref == 0.0:
            self.reset()
            return 0.0
        if w_gyro is None or gyro_age > self.gyro_timeout:
            self.reset()
            return math.copysign(clamp(abs(w_ref), self.min_spin, self.max_out), w_ref)
        if self.integral * w_ref < 0.0:  # direction flipped
            self.integral = 0.0
        dt = 0.0 if self.last_t is None else clamp(now - self.last_t, 0.0, 0.2)
        self.last_t = now
        err = w_ref - w_gyro
        out = w_ref + self.kp * err + self.integral
        saturated = abs(out) >= self.max_out and out * err > 0.0
        if not saturated:
            self.integral += self.ki * dt * err
            # the integrator only ever adds turn effort in the requested direction
            if self.integral * w_ref < 0.0:
                self.integral = 0.0
        return clamp(out, -self.max_out, self.max_out)


class Detector(threading.Thread):
    """Owns the OAK-D-Lite and keeps the latest detections."""

    def __init__(self, blob, fps, confidence, keep_aspect):
        super(Detector, self).__init__(daemon=True)
        self.blob, self.fps, self.confidence = blob, fps, confidence
        self.keep_aspect = keep_aspect
        self.preview_hfov = None  # rad, horizontal field of view of the NN input
        self.fy = None            # px, RGB focal length at 1080p (for person heights)
        self.lock = threading.Lock()
        self.latest = []      # list of (label, conf, x_m, z_m, bbox_cx, ymin, ymax)
        self.stamp = 0.0      # monotonic time of latest message
        self.error = None
        self.stop_flag = False

    def pipeline(self):
        p = dai.Pipeline()
        cam = p.create(dai.node.ColorCamera)
        cam.setPreviewSize(300, 300)
        # True: the preview is the centre 1080x1080 of the frame (narrower view, true shape).
        # False: the full width squashed to square (wider view, distorted shapes).
        cam.setPreviewKeepAspectRatio(self.keep_aspect)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(self.fps)
        left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
        for m, s in ((left, dai.CameraBoardSocket.LEFT), (right, dai.CameraBoardSocket.RIGHT)):
            m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            m.setBoardSocket(s)
            m.setFps(self.fps)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(left.getResolutionWidth(), left.getResolutionHeight())
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        nn = p.create(dai.node.MobileNetSpatialDetectionNetwork)
        nn.setBlobPath(self.blob)
        nn.setConfidenceThreshold(self.confidence)
        nn.input.setBlocking(False)
        nn.setBoundingBoxScaleFactor(0.5)
        nn.setDepthLowerThreshold(100)
        nn.setDepthUpperThreshold(8000)
        cam.preview.link(nn.input)
        stereo.depth.link(nn.inputDepth)
        xout = p.create(dai.node.XLinkOut)
        xout.setStreamName("det")
        nn.out.link(xout.input)
        return p

    def run(self):
        while not self.stop_flag:
            try:
                with dai.Device(self.pipeline()) as dev:
                    rospy.loginfo("OAK-D-Lite up (usb %s)", dev.getUsbSpeed().name)
                    self.preview_hfov = self.read_preview_hfov(dev)
                    q = dev.getOutputQueue("det", 4, False)
                    while not self.stop_flag:
                        msg = q.tryGet()
                        if msg is None:
                            time.sleep(0.01)
                            continue
                        dets = []
                        for d in msg.detections:
                            lab = LABELS[d.label] if d.label < len(LABELS) else str(d.label)
                            dets.append((lab, d.confidence, d.spatialCoordinates.x / 1000.0,
                                         d.spatialCoordinates.z / 1000.0, (d.xmin + d.xmax) / 2.0,
                                         d.ymin, d.ymax))
                        with self.lock:
                            self.latest, self.stamp, self.error = dets, time.monotonic(), None
            except Exception as e:  # camera unplugged / XLink error: retry
                self.error = str(e)
                rospy.logwarn_throttle(10, "OAK-D-Lite error, retrying: %s" % e)
                time.sleep(2.0)

    def read_preview_hfov(self, dev):
        """Horizontal field of view of the 300x300 preview, from the RGB calibration."""
        try:
            k = dev.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1920, 1080)
            fx = float(k[0][0])
            self.fy = float(k[1][1])
        except Exception as e:
            rospy.logwarn("No RGB intrinsics (%s); using rgb_hfov_deg", e)
            return None
        width_px = 1080.0 if self.keep_aspect else 1920.0
        hfov = 2.0 * math.atan(width_px / 2.0 / fx)
        rospy.loginfo("Detector sees %.1f deg horizontally (fx=%.0f px at 1080p, %s)",
                      math.degrees(hfov), fx, "centre crop" if self.keep_aspect else "full width")
        return hfov

    def get(self):
        with self.lock:
            return list(self.latest), self.stamp


class Chase(object):
    def __init__(self):
        gp = rospy.get_param
        self.dry_run = gp("~dry_run", True)
        self.arm_mode = gp("~arm_mode", "button")        # "button" or "auto"
        self.targets = self.parse_targets(gp("~targets", ["dog"]))
        self.kid_max_height = gp("~kid_max_height", 1.35)  # m, head height splitting kid/adult
        self.camera_height = gp("~camera_height", 0.15)  # m, lens above the floor
        self.camera_pitch = math.radians(gp("~camera_pitch_deg", 0.0))  # + = tilted up
        self.edge_margin = gp("~edge_margin", 0.02)      # box top this close to the frame top = clipped
        self.default_fy = gp("~rgb_fy_px", 1500.0)       # until the calibration is read
        self.button_chase = gp("~button_chase", 7)       # R2 (hid-sony js numbering)
        self.button_deadman = gp("~button_teleop", 4)    # L1, teleop owns it
        self.joy_timeout = gp("~joy_timeout", 0.5)
        self.lost_timeout = gp("~lost_timeout", 0.5)
        self.min_hits = gp("~min_hits", 3)               # consecutive frames before acting
        self.follow_distance = gp("~follow_distance", 0.9)
        self.stop_distance = gp("~stop_distance", 0.5)
        self.k_linear = gp("~k_linear", 0.4)             # (m/s) per m of range error
        self.max_linear = gp("~max_linear", 0.2)
        self.k_angular = gp("~k_angular", 2.5)           # (rad/s) per rad of bearing
        self.max_angular = gp("~max_angular", 2.0)       # requested yaw rate limit
        self.bearing_deadband = math.radians(gp("~bearing_deadband_deg", 5.0))
        self.full_speed_bearing = math.radians(gp("~full_speed_bearing_deg", 25.0))
        self.smoothing = gp("~smoothing", 0.5)           # EMA weight of the new sample
        self.hfov = math.radians(gp("~rgb_hfov_deg", 39.0))  # used until the calibration is read
        self.rate = gp("~rate", 15.0)
        self.log_status = gp("~log_status", True)        # status line to the console every 2 s

        self.yaw_loop_enabled = gp("~yaw_loop", True)
        self.yaw_loop = YawRateLoop(gp("~yaw_kp", 1.0), gp("~yaw_ki", 8.0),
                                    gp("~max_turn_cmd", 8.0), gp("~min_spin_cmd", 4.0),
                                    gp("~gyro_timeout", 0.3))
        self.gyro_z, self.gyro_stamp = None, 0.0
        self.camera_latency = gp("~camera_latency", 0.15)  # s, capture to host
        self.gyro_yaw, self.gyro_yaw_t = 0.0, None        # integrated gyro z, left +
        self.yaw_hist = collections.deque(maxlen=60)      # (monotonic, gyro_yaw), ~2 s

        self.det = Detector(gp("~blob", "/home/debian/models/mobilenet-ssd_openvino_2021.4_6shave.blob"),
                            gp("~fps", 10), gp("~confidence", 0.5),
                            gp("~preview_keep_aspect", True))
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.preview_pub = rospy.Publisher("chase/cmd_vel_preview", Twist, queue_size=1)
        self.target_pub = rospy.Publisher("chase/target", PointStamped, queue_size=1)
        self.status_pub = rospy.Publisher("chase/status", String, queue_size=1)
        self.active_pub = rospy.Publisher("chase/active", Bool, queue_size=1)
        self.targets_pub = rospy.Publisher("chase/targets", String, queue_size=1, latch=True)
        self.targets_pub.publish(",".join(self.targets))
        rospy.Subscriber("chase/set_targets", String, self.on_set_targets, queue_size=1)
        rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)
        rospy.Subscriber(gp("~imu_topic", "imu/data_raw"), Imu, self.on_imu, queue_size=1)

        self.buttons, self.joy_stamp = [], 0.0
        self.moving = False
        self.hits = 0
        self.track = None          # smoothed (azimuth_rad, range_m or None, label, conf);
                                   # azimuth = bearing - gyro yaw, fixed in the room
        self.track_stamp = 0.0
        self.track_height = None   # m, estimated height of a tracked person
        self.last_frame = 0.0

    @staticmethod
    def parse_targets(t):
        if isinstance(t, str):
            t = t.replace("[", "").replace("]", "").split(",")
        return [str(x).strip().lower() for x in t if str(x).strip()]

    def on_set_targets(self, msg):
        t = self.parse_targets(msg.data)
        if not t:
            rospy.logwarn("chase: empty target list ignored")
            return
        self.targets = t
        self.hits = 0
        self.targets_pub.publish(",".join(t))
        rospy.loginfo("chase: targets now %s", t)

    def height_at(self, z, y):
        """Height above the floor (m) of image row y (0 top .. 1 bottom) at depth z."""
        fy = self.det.fy or self.default_fy
        # the preview is 1080 px tall in both crop modes (1080p sensor)
        angle = math.atan((0.5 - y) * 1080.0 / fy) + self.camera_pitch
        return self.camera_height + z * math.tan(angle)

    def person_class(self, z, ymin):
        """('kid' | 'adult' | 'person', head height in m or None) for a person box."""
        if z <= 0.05:
            return "person", None
        if ymin < self.edge_margin:   # head above the frame: frame top is a lower bound
            h = self.height_at(z, 0.0)
            return ("adult" if h > self.kid_max_height else "person"), None
        h = self.height_at(z, ymin)
        return ("kid" if h < self.kid_max_height else "adult"), h

    def matches(self, d):
        """(label to report, height_m) if detection d is a target, else None."""
        lab = d[0]
        if lab != "person":
            return (lab, None) if lab in self.targets else None
        cls, h = self.person_class(d[3], d[5])
        if "person" in self.targets or cls in self.targets:
            return cls, h
        if cls == "person" and "kid" in self.targets and self.hits > 0 \
                and self.track is not None and self.track[2] == "kid":
            return "kid", h  # can't see the head now, but we were following a kid
        return None

    def on_joy(self, msg):
        self.buttons, self.joy_stamp = list(msg.buttons), time.monotonic()

    def on_imu(self, msg):
        now = time.monotonic()
        t = msg.header.stamp.to_sec()
        if self.gyro_yaw_t is not None and 0.0 < t - self.gyro_yaw_t < 0.5:
            self.gyro_yaw += msg.angular_velocity.z * (t - self.gyro_yaw_t)
        self.gyro_yaw_t = t
        self.gyro_z, self.gyro_stamp = msg.angular_velocity.z, now
        self.yaw_hist.append((now, self.gyro_yaw))

    def yaw_at(self, t):
        """Gyro yaw at monotonic time t (latest sample at or before it)."""
        hist = list(self.yaw_hist)
        yaw = hist[0][1] if hist else self.gyro_yaw
        for ts, y in hist:
            if ts > t:
                break
            yaw = y
        return yaw

    def bearing(self):
        """Current bearing of the tracked target (rad, + right). A left turn moves a
        fixed target to the right by the same angle."""
        return self.track[0] + self.gyro_yaw

    def held(self, i):
        return i < len(self.buttons) and self.buttons[i] == 1

    def armed(self):
        fresh = time.monotonic() - self.joy_stamp < self.joy_timeout
        if self.arm_mode == "auto":
            # no controller connected is fine; L1 held means teleop is driving
            return not (fresh and self.held(self.button_deadman))
        return fresh and self.held(self.button_chase) and not self.held(self.button_deadman)

    def stop(self):
        if self.moving:
            if not self.dry_run:
                self.cmd_pub.publish(Twist())
            self.preview_pub.publish(Twist())
            self.moving = False

    def update_track(self):
        dets, stamp = self.det.get()
        if stamp == self.last_frame:
            return
        self.last_frame = stamp
        cands = []
        for d in dets:
            m = self.matches(d)
            if m is not None:
                cands.append((m[0], d[1], d[2], d[3], d[4], m[1]))
        if not cands:
            self.hits = 0
            return
        lab, conf, x, z, cx, self.track_height = max(cands, key=lambda d: d[1])
        yaw_capture = self.yaw_at(stamp - self.camera_latency)
        if z > 0.05:
            bearing, rng = math.atan2(x, z), math.hypot(x, z)
        else:  # no valid depth in the box: bearing from the image, range unknown
            hfov = self.det.preview_hfov or self.hfov
            bearing, rng = math.atan((2.0 * cx - 1.0) * math.tan(hfov / 2.0)), None
        azimuth = bearing - yaw_capture
        a = self.smoothing
        if self.track is None or self.hits == 0:
            sb, sr = azimuth, rng
        else:
            pb, pr = self.track[0], self.track[1]
            sb = a * azimuth + (1 - a) * pb
            sr = rng if pr is None else (pr if rng is None else a * rng + (1 - a) * pr)
        self.track, self.track_stamp = (sb, sr, lab, conf), stamp
        self.hits += 1

        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "oak_rgb"
        pt.point.x = (sr or 0.0) * math.sin(self.bearing())
        pt.point.z = (sr or 0.0) * math.cos(self.bearing())
        self.target_pub.publish(pt)

    def turn_command(self, w_ref, armed):
        """angular.z to send for a requested yaw rate w_ref (rad/s)."""
        if not self.yaw_loop_enabled:
            return w_ref
        if not armed:
            # nothing is driving, so the gyro says nothing about the floor yet
            self.yaw_loop.reset()
            return w_ref
        now = time.monotonic()
        return self.yaw_loop.update(w_ref, self.gyro_z, now - self.gyro_stamp, now)

    def command(self):
        """Twist toward the current track. None means no usable target."""
        if self.track is None or self.hits < self.min_hits:
            return None
        if time.monotonic() - self.track_stamp > self.lost_timeout:
            return None
        bearing, rng = self.bearing(), self.track[1]
        t = Twist()
        if abs(bearing) > self.bearing_deadband:
            # bearing + = target to the right; ROS angular.z + = turn left
            t.angular.z = clamp(-self.k_angular * bearing, -self.max_angular, self.max_angular)
        if rng is not None and rng > self.stop_distance:
            v = clamp(self.k_linear * (rng - self.follow_distance), 0.0, self.max_linear)
            v *= clamp(1.0 - abs(bearing) / self.full_speed_bearing, 0.0, 1.0)
            t.linear.x = v
        return t

    def spin(self):
        self.det.start()
        r = rospy.Rate(self.rate)
        last_status = 0.0
        while not rospy.is_shutdown():
            self.update_track()
            cmd = self.command()
            armed = self.armed()
            w_ref = 0.0
            if cmd is not None:
                w_ref = cmd.angular.z
                cmd.angular.z = self.turn_command(w_ref, armed)
            else:
                self.yaw_loop.reset()

            self.active_pub.publish(cmd is not None)
            if cmd is not None:
                self.preview_pub.publish(cmd)
            if armed and cmd is not None:
                if not self.dry_run:
                    self.cmd_pub.publish(cmd)
                self.moving = True
            else:
                self.stop()

            now = time.monotonic()
            if now - last_status > 0.5:
                if self.det.error:
                    s = "CAMERA_ERROR " + self.det.error[:60]
                elif cmd is None:
                    s = "SEARCH"
                else:
                    _, rng, lab, conf = self.track
                    b = self.bearing()
                    gyro = "%.2f" % self.gyro_z if self.gyro_z is not None else "?"
                    if self.track_height is not None:
                        lab += "(%.2fm tall)" % self.track_height
                    s = "TRACK %s %d%% %s %+.1fdeg v=%.2f w=%.2f->%.2f gyro=%s" % (
                        lab, conf * 100, "%.2fm" % rng if rng else "?m", math.degrees(b),
                        cmd.linear.x, w_ref, cmd.angular.z, gyro)
                s += " | %s%s%s" % ("ARMED" if armed else "idle", " auto" if self.arm_mode == "auto" else "",
                                    " (dry run)" if self.dry_run else "")
                self.status_pub.publish(s)
                if self.log_status:
                    rospy.loginfo_throttle(2.0, s)
                last_status = now
            r.sleep()

    def shutdown(self):
        self.det.stop_flag = True
        self.moving = True  # force a final zero
        self.stop()


if __name__ == "__main__":
    import os
    rospy.init_node("chase")
    node = Chase()
    rospy.on_shutdown(node.shutdown)
    rospy.loginfo("chase: targets=%s dry_run=%s arm_mode=%s (%s, L1 teleop overrides)",
                  node.targets, node.dry_run, node.arm_mode,
                  "drives whenever a target is tracked" if node.arm_mode == "auto" else "hold R2 to arm")
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    # depthai 2.19 can hang in its destructors on this kernel; give the camera thread a
    # moment to close the device, then exit hard so roslaunch never has to escalate.
    node.det.join(3.0)
    os._exit(0)
