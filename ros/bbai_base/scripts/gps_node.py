#!/usr/bin/env python3
"""Publish the EM-506 GPS (NMEA over a UART) as sensor_msgs/NavSatFix.

Topics: /gps/fix (NavSatFix, every GGA sentence; status NO_FIX without a fix)
        /gps/vel (TwistStamped, ENU ground velocity from RMC, only with a fix)
Covariance comes from HDOP times ~uere (meters), so a poor fix is trusted less.
"""
import math
import os
import termios

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

BAUDS = {4800: termios.B4800, 9600: termios.B9600, 38400: termios.B38400}
KNOT = 0.514444


def open_serial(port, baud):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = termios.IGNPAR
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = termios.ICANON  # line at a time
    a[4] = a[5] = BAUDS[baud]
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return os.fdopen(fd, "r", errors="ignore")


def checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, cs = line[1:].split("*", 1)
    calc = 0
    for c in body:
        calc ^= ord(c)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def degrees(value, hemi):
    if not value:
        return float("nan")
    dot = value.index(".")
    deg = float(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -deg if hemi in ("S", "W") else deg


class GpsNode(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyS2")
        self.baud = int(rospy.get_param("~baud", 4800))
        self.frame = rospy.get_param("~frame_id", "gps_link")
        self.uere = float(rospy.get_param("~uere", 5.0))
        self.fix_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=10)
        self.vel_pub = rospy.Publisher("gps/vel", TwistStamped, queue_size=10)
        self.has_fix = False

    def on_gga(self, f):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.status.service = NavSatStatus.SERVICE_GPS
        quality = int(f[6] or 0)
        self.has_fix = quality > 0 and bool(f[2])
        if not self.has_fix:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = float("nan")
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.status.status = (NavSatStatus.STATUS_SBAS_FIX if quality == 2
                                 else NavSatStatus.STATUS_FIX)
            msg.latitude = degrees(f[2], f[3])
            msg.longitude = degrees(f[4], f[5])
            msg.altitude = float(f[9] or "nan") + float(f[11] or 0.0)  # ellipsoid height
            hdop = float(f[8] or 99.0)
            var_h = (hdop * self.uere) ** 2
            msg.position_covariance = [var_h, 0, 0, 0, var_h, 0, 0, 0, 4 * var_h]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(msg)

    def on_rmc(self, f):
        if f[2] != "A" or not self.has_fix or not f[7]:
            return
        speed = float(f[7]) * KNOT
        course = math.radians(float(f[8] or 0.0))  # clockwise from north
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.twist.linear.x = speed * math.sin(course)  # east
        msg.twist.linear.y = speed * math.cos(course)  # north
        self.vel_pub.publish(msg)

    def spin(self):
        ser = open_serial(self.port, self.baud)
        rospy.loginfo("Reading NMEA from %s at %d baud", self.port, self.baud)
        while not rospy.is_shutdown():
            line = ser.readline().strip()
            if not checksum_ok(line):
                continue
            f = line.split("*")[0].split(",")
            try:
                if f[0].endswith("GGA") and len(f) >= 12:
                    self.on_gga(f)
                elif f[0].endswith("RMC") and len(f) >= 9:
                    self.on_rmc(f)
            except ValueError as e:
                rospy.logwarn_throttle(10, "Bad NMEA sentence %r: %s", line, e)


if __name__ == "__main__":
    rospy.init_node("gps")
    GpsNode().spin()
