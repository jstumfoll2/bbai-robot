#!/usr/bin/env python3
"""Calibrate the compass by spinning the robot in place on the floor.

Needs bbai_base running. Drives /cmd_vel to turn about two full circles,
records /imu/mag and the gyro, then fits a circle to the horizontal field
(hard-iron offset) and checks the heading turns the same way as the gyro.
Prints the base.yaml lines to use.
"""
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField

TURN_RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0  # rad/s command; lower stalls on the floor
TARGET = 2 * 2 * math.pi  # two turns, measured by the gyro
MAX_TIME = 40.0

mags, gyro_yaw = [], [0.0]
last_gyro_t = [None]


def on_mag(m):
    mags.append((m.magnetic_field.x * 1e6, m.magnetic_field.y * 1e6, gyro_yaw[0]))


def on_imu(m):
    t = m.header.stamp.to_sec()
    if last_gyro_t[0] is not None:
        gyro_yaw[0] += m.angular_velocity.z * (t - last_gyro_t[0])
    last_gyro_t[0] = t


def fit_circle(pts):
    # Least squares x^2 + y^2 + D x + E y + F = 0
    sxx = sxy = syy = sx = sy = n = 0.0
    sxz = syz = sz = 0.0
    for x, y in pts:
        z = x * x + y * y
        sxx += x * x; sxy += x * y; syy += y * y
        sx += x; sy += y; n += 1
        sxz += x * z; syz += y * z; sz += z
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    b = [-sxz, -syz, -sz]
    # Solve 3x3 by Cramer's rule
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(a)
    sol = []
    for i in range(3):
        mi = [row[:] for row in a]
        for r in range(3):
            mi[r][i] = b[r]
        sol.append(det(mi) / d)
    D, E, F = sol
    cx, cy = -D / 2, -E / 2
    return cx, cy, math.sqrt(cx * cx + cy * cy - F)


def main():
    rospy.init_node("mag_calibrate")
    rospy.Subscriber("imu/mag", MagneticField, on_mag)
    rospy.Subscriber("imu/data_raw", Imu, on_imu)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    time.sleep(1.5)
    cmd = Twist()
    cmd.angular.z = TURN_RATE
    start = time.time()
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown() and abs(gyro_yaw[0]) < TARGET:
            if time.time() - start > MAX_TIME:
                print("Timed out after %.0f s, gyro saw %.0f deg" % (MAX_TIME, math.degrees(gyro_yaw[0])))
                break
            pub.publish(cmd)
            rate.sleep()
    finally:
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.05)

    if len(mags) < 50:
        print("Only %d magnetometer samples, aborting" % len(mags))
        return
    cx, cy, r = fit_circle([(x, y) for x, y, _ in mags])
    # Heading change from the compass vs the gyro, to get the sign right
    unwrapped, prev = 0.0, None
    for x, y, _ in mags:
        h = math.atan2(x - cx, y - cy)
        if prev is not None:
            unwrapped += math.atan2(math.sin(h - prev), math.cos(h - prev))
        prev = h
    gyro_turn = mags[-1][2] - mags[0][2]
    sign = 1.0 if unwrapped * gyro_turn > 0 else -1.0
    spread = [abs(math.hypot(x - cx, y - cy) - r) for x, y, _ in mags]
    print("samples %d, gyro turned %.0f deg, compass turned %.0f deg"
          % (len(mags), math.degrees(gyro_turn), math.degrees(unwrapped)))
    print("field radius %.1f uT, fit error %.1f uT mean" % (r, sum(spread) / len(spread)))
    print("\nmag_heading: true\nmag_offset_x: %.2f\nmag_offset_y: %.2f\nmag_sign: %.0f" % (cx, cy, sign))


if __name__ == "__main__":
    main()
