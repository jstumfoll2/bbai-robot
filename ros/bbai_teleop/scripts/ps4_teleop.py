#!/usr/bin/env python3
"""PS4 (DualShock 4) teleop for the BeagleBone AI robot.

Reads the Linux joystick API (/dev/input/jsN, created by hid-sony when the
controller connects over Bluetooth or USB) and publishes:
  /joy      sensor_msgs/Joy   raw axes and buttons
  /cmd_vel  geometry_msgs/Twist, only while the deadman button is held

Default mapping (hid-sony on kernel 4.14):
  hold L1            deadman, nothing moves unless it is held
  left stick up/down linear.x
  right stick l/r    angular.z
  hold R1            turbo (scale_*_turbo instead of scale_*)
Releasing L1 or losing the controller publishes one zero Twist.
"""
import glob
import os
import select
import struct

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

JS_EVENT = struct.Struct("IhBB")  # time ms, value, type, number
JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80


def find_device(name_match):
    """Return the first /dev/input/jsN whose device name contains name_match."""
    for path in sorted(glob.glob("/dev/input/js*")):
        sysname = "/sys/class/input/%s/device/name" % os.path.basename(path)
        try:
            with open(sysname) as f:
                name = f.read().strip()
        except IOError:
            continue
        if name_match.lower() in name.lower() and "motion" not in name.lower():
            return path, name
    return None, None


class Ps4Teleop(object):
    def __init__(self):
        self.name_match = rospy.get_param("~device_name", "Wireless Controller")
        self.device = rospy.get_param("~device", "")  # fixed path overrides name search
        self.deadzone = rospy.get_param("~deadzone", 0.08)
        self.axis_linear = rospy.get_param("~axis_linear", 1)
        self.axis_angular = rospy.get_param("~axis_angular", 3)
        self.button_deadman = rospy.get_param("~button_deadman", 4)
        self.button_turbo = rospy.get_param("~button_turbo", 5)
        self.scale_linear = rospy.get_param("~scale_linear", 0.3)  # m/s
        self.scale_angular = rospy.get_param("~scale_angular", 1.5)  # rad/s
        self.scale_linear_turbo = rospy.get_param("~scale_linear_turbo", 0.6)
        self.scale_angular_turbo = rospy.get_param("~scale_angular_turbo", 3.0)
        self.rate = rospy.get_param("~rate", 20.0)  # heartbeat while sticks are idle
        self.min_interval = 1.0 / rospy.get_param("~max_rate", 100.0)  # cap when sticks move

        self.joy_pub = rospy.Publisher("joy", Joy, queue_size=1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.axes = []
        self.buttons = []
        self.moving = False

    def axis(self, i):
        v = self.axes[i] if i < len(self.axes) else 0.0
        return 0.0 if abs(v) < self.deadzone else v

    def button(self, i):
        return i < len(self.buttons) and self.buttons[i] == 1

    def stop(self):
        if self.moving:
            self.cmd_pub.publish(Twist())
            self.moving = False

    def publish(self):
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.axes = list(self.axes)
        joy.buttons = list(self.buttons)
        self.joy_pub.publish(joy)

        if not self.button(self.button_deadman):
            self.stop()
            return
        turbo = self.button(self.button_turbo)
        t = Twist()
        # Stick up is negative on the joystick API, so flip it for forward
        t.linear.x = -self.axis(self.axis_linear) * (
            self.scale_linear_turbo if turbo else self.scale_linear)
        t.angular.z = -self.axis(self.axis_angular) * (
            self.scale_angular_turbo if turbo else self.scale_angular)
        self.cmd_pub.publish(t)
        self.moving = True

    def handle(self, data):
        _, value, etype, number = JS_EVENT.unpack(data)
        etype &= ~JS_INIT
        if etype == JS_AXIS:
            while len(self.axes) <= number:
                self.axes.append(0.0)
            self.axes[number] = value / 32767.0
        elif etype == JS_BUTTON:
            while len(self.buttons) <= number:
                self.buttons.append(0)
            self.buttons[number] = 1 if value else 0

    def run_device(self, path):
        period = 1.0 / self.rate
        next_pub = last_pub = rospy.get_time()
        fd =os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            while not rospy.is_shutdown():
                timeout = max(0.0, next_pub - rospy.get_time())
                ready, _, _ = select.select([fd], [], [], timeout)
                if ready:
                    # Drain everything queued so the sticks never lag behind a backlog
                    try:
                        data = os.read(fd, JS_EVENT.size * 64)
                    except BlockingIOError:
                        data = None
                    if data == b"":
                        raise IOError("device closed")
                    if data:
                        for i in range(0, len(data) - JS_EVENT.size + 1, JS_EVENT.size):
                            self.handle(data[i:i + JS_EVENT.size])
                        # React to stick changes right away instead of waiting for the next tick
                        next_pub = min(next_pub, last_pub + self.min_interval)
                now = rospy.get_time()
                if now >= next_pub:
                    self.publish()
                    last_pub = now
                    next_pub = now + period
        finally:
            os.close(fd)

    def spin(self):
        while not rospy.is_shutdown():
            path, name = (self.device, self.device) if self.device else find_device(self.name_match)
            if not path or not os.path.exists(path):
                rospy.loginfo_throttle(10, "Waiting for a '%s' joystick..." % self.name_match)
                rospy.sleep(1.0)
                continue
            rospy.loginfo("Using %s (%s)", path, name)
            self.axes, self.buttons = [], []
            try:
                self.run_device(path)
            except (IOError, OSError) as e:
                rospy.logwarn("Lost joystick %s: %s", path, e)
            self.stop()
            rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("ps4_teleop")
    node = Ps4Teleop()
    rospy.on_shutdown(node.stop)
    node.spin()
