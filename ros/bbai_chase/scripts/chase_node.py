#!/usr/bin/env python3
"""Chase-the-dog node for the BeagleBone AI robot.

Runs MobileNet-SSD spatial detection on the OAK-D-Lite (all inference on the
camera's Myriad X) and, while the chase button is held on the PS4 controller,
steers toward the best target and drives up to a follow distance.

Topics
  sub  /joy                   sensor_msgs/Joy (from bbai_teleop)
  pub  /chase/target          geometry_msgs/PointStamped, camera frame: x right, z forward (m)
  pub  /chase/status          std_msgs/String, e.g. "TRACK dog 92% 1.40m -8.2deg"
  pub  /cmd_vel               geometry_msgs/Twist, only while armed (dry_run:=false)
  pub  /chase/cmd_vel_preview geometry_msgs/Twist, what it would send (always)

Safety
  - Moves only while the chase button is held AND the teleop deadman (L1) is not.
    Teleop always wins.
  - One zero Twist on release, on target loss (> lost_timeout), on joy timeout, on exit.
  - No reverse. Stops inside stop_distance. Speed capped by max_linear / max_angular.
  - dry_run (default true) never publishes /cmd_vel. It only fills the preview topic.
"""
import math
import threading
import time

# Import order matters: on this board, loading numpy's C extension (pulled in by
# rospy/genpy) *after* depthai's native library deadlocks in dlopen. Keep numpy and
# rospy above depthai.
import numpy  # noqa: F401
import rospy
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import depthai as dai

LABELS = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
          "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
          "sheep", "sofa", "train", "tvmonitor"]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Detector(threading.Thread):
    """Owns the OAK-D-Lite and keeps the latest detections."""

    def __init__(self, blob, fps, confidence):
        super(Detector, self).__init__(daemon=True)
        self.blob, self.fps, self.confidence = blob, fps, confidence
        self.lock = threading.Lock()
        self.latest = []      # list of (label, conf, x_m, z_m, bbox_cx)
        self.stamp = 0.0      # monotonic time of latest message
        self.error = None
        self.stop_flag = False

    def pipeline(self):
        p = dai.Pipeline()
        cam = p.create(dai.node.ColorCamera)
        cam.setPreviewSize(300, 300)
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
                                         d.spatialCoordinates.z / 1000.0, (d.xmin + d.xmax) / 2.0))
                        with self.lock:
                            self.latest, self.stamp, self.error = dets, time.monotonic(), None
            except Exception as e:  # camera unplugged / XLink error: retry
                self.error = str(e)
                rospy.logwarn_throttle(10, "OAK-D-Lite error, retrying: %s" % e)
                time.sleep(2.0)

    def get(self):
        with self.lock:
            return list(self.latest), self.stamp


class Chase(object):
    def __init__(self):
        gp = rospy.get_param
        self.dry_run = gp("~dry_run", True)
        self.targets = gp("~targets", ["dog"])
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
        self.max_angular = gp("~max_angular", 2.0)
        self.bearing_deadband = math.radians(gp("~bearing_deadband_deg", 5.0))
        self.full_speed_bearing = math.radians(gp("~full_speed_bearing_deg", 25.0))
        self.smoothing = gp("~smoothing", 0.5)           # EMA weight of the new sample
        self.hfov = math.radians(gp("~rgb_hfov_deg", 68.8))
        self.rate = gp("~rate", 15.0)
        self.log_status = gp("~log_status", True)        # status line to the console every 2 s

        self.det = Detector(gp("~blob", "/home/debian/models/mobilenet-ssd_openvino_2021.4_6shave.blob"),
                            gp("~fps", 10), gp("~confidence", 0.5))
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.preview_pub = rospy.Publisher("chase/cmd_vel_preview", Twist, queue_size=1)
        self.target_pub = rospy.Publisher("chase/target", PointStamped, queue_size=1)
        self.status_pub = rospy.Publisher("chase/status", String, queue_size=1)
        rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)

        self.buttons, self.joy_stamp = [], 0.0
        self.moving = False
        self.hits = 0
        self.track = None          # smoothed (bearing_rad, range_m or None, label, conf)
        self.track_stamp = 0.0
        self.last_frame = 0.0

    def on_joy(self, msg):
        self.buttons, self.joy_stamp = list(msg.buttons), time.monotonic()

    def held(self, i):
        return i < len(self.buttons) and self.buttons[i] == 1

    def armed(self):
        fresh = time.monotonic() - self.joy_stamp < self.joy_timeout
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
        cands = [d for d in dets if d[0] in self.targets]
        if not cands:
            self.hits = 0
            return
        lab, conf, x, z, cx = max(cands, key=lambda d: d[1])
        if z > 0.05:
            bearing, rng = math.atan2(x, z), math.hypot(x, z)
        else:  # no valid depth in the box: bearing from the image, range unknown
            bearing, rng = (cx - 0.5) * self.hfov, None
        a = self.smoothing
        if self.track is None or self.hits == 0:
            sb, sr = bearing, rng
        else:
            pb, pr = self.track[0], self.track[1]
            sb = a * bearing + (1 - a) * pb
            sr = rng if pr is None else (pr if rng is None else a * rng + (1 - a) * pr)
        self.track, self.track_stamp = (sb, sr, lab, conf), stamp
        self.hits += 1

        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "oak_rgb"
        pt.point.x = (sr or 0.0) * math.sin(sb)
        pt.point.z = (sr or 0.0) * math.cos(sb)
        self.target_pub.publish(pt)

    def command(self):
        """Twist toward the current track. None means no usable target."""
        if self.track is None or self.hits < self.min_hits:
            return None
        if time.monotonic() - self.track_stamp > self.lost_timeout:
            return None
        bearing, rng = self.track[0], self.track[1]
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
                    b, rng, lab, conf = self.track
                    s = "TRACK %s %d%% %s %+.1fdeg v=%.2f w=%.2f" % (
                        lab, conf * 100, "%.2fm" % rng if rng else "?m", math.degrees(b),
                        cmd.linear.x, cmd.angular.z)
                s += " | %s%s" % ("ARMED" if armed else "idle", " (dry run)" if self.dry_run else "")
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
    rospy.loginfo("chase: targets=%s dry_run=%s (hold R2 to arm, L1 teleop overrides)",
                  node.targets, node.dry_run)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    # depthai 2.19 can hang in its destructors on this kernel; give the camera thread a
    # moment to close the device, then exit hard so roslaunch never has to escalate.
    node.det.join(3.0)
    os._exit(0)
