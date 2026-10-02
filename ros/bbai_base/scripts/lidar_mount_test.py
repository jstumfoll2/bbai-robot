#!/usr/bin/env python3
"""Check the lidar's mounting direction against odometry.

Needs bbai_base and rplidar running and room around the robot. Turns about
90 degrees in place, then drives about 0.3 m forward. For the turn it finds
the circular shift that best lines up the before/after scans (normal and
mirrored); for the drive it finds which laser bearing got closer.
"""
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

state = {}


def yaw(o):
    q = o.pose.pose.orientation
    return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)


def grab_scan():
    time.sleep(1.0)
    return rospy.wait_for_message("scan", LaserScan, 5)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def resample(scan, n=360):
    """Ranges on a fixed 1-degree grid in the laser frame (nan where empty)."""
    out = [float("nan")] * n
    for i, r in enumerate(scan.ranges):
        if math.isinf(r) or math.isnan(r) or r < scan.range_min:
            continue
        a = scan.angle_min + i * scan.angle_increment
        k = int(round(math.degrees(a))) % n
        out[k] = r
    return out


def best_shift(a, b, mirror=False):
    n = len(a)
    best = (1e9, 0)
    for s in range(n):
        err = cnt = 0
        for k in range(n):
            j = (-k if mirror else k) % n
            x, y = a[k], b[(j + s) % n]
            if x == x and y == y:
                err += min(abs(x - y), 0.5)
                cnt += 1
        if cnt > 100:
            best = min(best, (err / cnt, s))
    return best


def drive(pub, lin, ang, until):
    rate = rospy.Rate(20)
    start = time.time()
    while not rospy.is_shutdown() and not until() and time.time() - start < 15:
        t = Twist()
        t.linear.x, t.angular.z = lin, ang
        pub.publish(t)
        rate.sleep()
    for _ in range(5):
        pub.publish(Twist())
        time.sleep(0.05)


def main():
    rospy.init_node("lidar_mount_test")
    rospy.Subscriber("odom", Odometry, lambda m: state.__setitem__("odom", m))
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    time.sleep(2)

    s0, y0 = resample(grab_scan()), yaw(state["odom"])
    drive(pub, 0.0, 5.0, lambda: abs(wrap(yaw(state["odom"]) - y0)) > math.radians(85))
    s1, y1 = resample(grab_scan()), yaw(state["odom"])
    dyaw = math.degrees(wrap(y1 - y0))
    e_n, sh_n = best_shift(s0, s1)
    e_m, sh_m = best_shift(s0, s1, mirror=True)
    shift = sh_n if sh_n <= 180 else sh_n - 360
    print("TURN: odom turned %+.1f deg" % dyaw)
    print("  scan shift %+d deg (match error %.3f m); mirrored match error %.3f m"
          % (shift, e_n, e_m))
    print("  expected scan shift for a correctly mounted lidar: %+.0f deg" % -dyaw)

    o0 = state["odom"].pose.pose.position
    d0 = resample(grab_scan())
    drive(pub, 0.25, 0.0, lambda: math.hypot(state["odom"].pose.pose.position.x - o0.x,
                                             state["odom"].pose.pose.position.y - o0.y) > 0.3)
    o1 = state["odom"].pose.pose.position
    d1 = resample(grab_scan())
    moved = math.hypot(o1.x - o0.x, o1.y - o0.y)
    # Average range decrease in 30-degree sectors
    sectors = []
    for c in range(0, 360, 30):
        diffs = [d0[k % 360] - d1[k % 360] for k in range(c - 15, c + 15)
                 if d0[k % 360] == d0[k % 360] and d1[k % 360] == d1[k % 360]]
        if len(diffs) > 5:
            diffs.sort()
            sectors.append((diffs[len(diffs) // 2], c))
    sectors.sort(reverse=True)
    print("DRIVE: odom moved %.2f m forward" % moved)
    print("  laser bearings that got closest (median range drop, bearing):",
          ", ".join("%.2f m @ %d deg" % s for s in sectors[:3]))
    print("  for a correctly mounted lidar the front is 0 deg")


if __name__ == "__main__":
    main()
