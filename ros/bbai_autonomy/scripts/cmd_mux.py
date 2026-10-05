#!/usr/bin/env python3
"""Picks who drives the motors in autonomous mode. Runs on the board.

Priority, highest first:
  1. PS4 teleop: while L1 is held the mux publishes nothing; ps4_teleop owns /cmd_vel.
  2. Chase:      chase/cmd_vel, while its messages are fresh (a target is tracked).
  3. Navigation: nav/cmd_vel from move_base (exploring or patrolling).
Nothing passes while autonomy is disabled. Options on the PS4 controller toggles it,
and so does /autonomy/enable (std_msgs/Bool).

Topics
  sub  chase/cmd_vel, nav/cmd_vel   geometry_msgs/Twist
  sub  joy                          sensor_msgs/Joy
  sub  autonomy/enable              std_msgs/Bool
  pub  cmd_vel                      geometry_msgs/Twist
  pub  autonomy/enabled             std_msgs/Bool (latched)
  pub  autonomy/source              std_msgs/String (latched): off, teleop, chase, nav, idle

Safety: if the off-board host dies, nav/cmd_vel stops and bbai_base halts the motors
after its cmd_timeout. Every hand-over to "nobody" sends one zero Twist.
"""
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class CmdMux(object):
    def __init__(self):
        gp = rospy.get_param
        self.enabled = gp("~autostart", True)
        self.button_deadman = gp("~button_teleop", 4)   # L1
        self.button_toggle = gp("~button_toggle", 9)    # Options
        self.joy_timeout = gp("~joy_timeout", 0.5)
        self.chase_timeout = gp("~chase_timeout", 0.4)  # chase publishes at ~15 Hz
        self.nav_timeout = gp("~nav_timeout", 0.6)      # move_base publishes at ~5 Hz

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.enabled_pub = rospy.Publisher("autonomy/enabled", Bool, queue_size=1, latch=True)
        self.source_pub = rospy.Publisher("autonomy/source", String, queue_size=1, latch=True)
        self.buttons, self.joy_stamp = [], 0.0
        self.chase_stamp = self.nav_stamp = 0.0
        self.source = None
        self.set_source("idle" if self.enabled else "off")
        self.enabled_pub.publish(self.enabled)

        rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)
        rospy.Subscriber("autonomy/enable", Bool, lambda m: self.set_enabled(m.data), queue_size=1)
        rospy.Subscriber("chase/cmd_vel", Twist, self.on_chase, queue_size=1)
        rospy.Subscriber("nav/cmd_vel", Twist, self.on_nav, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.on_timer)

    def held(self, buttons, i):
        return i < len(buttons) and buttons[i] == 1

    def teleop(self):
        fresh = time.monotonic() - self.joy_stamp < self.joy_timeout
        return fresh and self.held(self.buttons, self.button_deadman)

    def set_enabled(self, on):
        if on == self.enabled:
            return
        self.enabled = on
        self.enabled_pub.publish(on)
        rospy.loginfo("autonomy %s", "ENABLED" if on else "DISABLED")
        if not on:
            self.handover("off")

    def set_source(self, src):
        if src != self.source:
            self.source = src
            self.source_pub.publish(src)

    def handover(self, src):
        """Switch source; stop the motors if nobody autonomous drives now."""
        prev = self.source
        self.set_source(src)
        if src in ("off", "idle") and prev in ("chase", "nav") and not self.teleop():
            self.cmd_pub.publish(Twist())

    def on_joy(self, msg):
        if self.held(msg.buttons, self.button_toggle) and not self.held(self.buttons, self.button_toggle):
            self.set_enabled(not self.enabled)
        self.buttons, self.joy_stamp = list(msg.buttons), time.monotonic()

    def allowed(self):
        if not self.enabled:
            return False
        if self.teleop():
            self.set_source("teleop")
            return False
        return True

    def on_chase(self, msg):
        self.chase_stamp = time.monotonic()
        if self.allowed():
            self.set_source("chase")
            self.cmd_pub.publish(msg)

    def on_nav(self, msg):
        if time.monotonic() - self.chase_stamp < self.chase_timeout:
            return  # chase has the wheels
        if self.allowed():
            self.set_source("nav")
            self.cmd_pub.publish(msg)
            self.nav_stamp = time.monotonic()

    def on_timer(self, _):
        now = time.monotonic()
        if not self.enabled:
            self.set_source("off")
        elif self.teleop():
            self.set_source("teleop")
        elif self.source == "teleop":
            self.set_source("idle")  # teleop sent its own zero on release
        elif self.source == "chase" and now - self.chase_stamp > self.chase_timeout:
            self.handover("idle")
        elif self.source == "nav" and now - self.nav_stamp > self.nav_timeout:
            self.handover("idle")


if __name__ == "__main__":
    rospy.init_node("cmd_mux")
    m = CmdMux()
    rospy.loginfo("cmd_mux: autonomy %s (Options toggles, L1 teleop overrides)",
                  "enabled" if m.enabled else "disabled")
    rospy.spin()
