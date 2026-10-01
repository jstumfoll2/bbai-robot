#!/usr/bin/env python3
"""Drive the motors and publish wheel odometry and IMU data from the Robotics Cape.

Subscribes: /cmd_vel (geometry_msgs/Twist), skid steer; motors stop if no
            message arrives within ~cmd_timeout seconds
Topics:  /odom (nav_msgs/Odometry), /imu/data_raw (sensor_msgs/Imu),
         /imu/mag (sensor_msgs/MagneticField), /wheel_ticks (raw counts ch1-4, debug)
TF:      odom -> base_link (when ~publish_tf is true)

Translation comes from the wheel encoders. Heading comes from the
integrated gyro z axis (bias measured at startup) when ~use_gyro_heading
is true, because the encoders give only a few counts per wheel turn.
"""
import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Int32MultiArray

import rcpy
import rcpy.encoder as encoder
import rcpy.mpu9250 as mpu9250

DEG2RAD = math.pi / 180.0

# Motor 1's B direction pin (P9.13) is only reachable from PRU2_0 on the
# BeagleBone AI. librobotcontrol drives it through the gpio_rpmsg firmware's
# /dev/rpmsg_pru32, so that firmware must be running before rc_motor_init.
PRU2_0 = "/sys/class/remoteproc/remoteproc2"


def start_motor_dir_pru():
    with open(PRU2_0 + "/state") as f:
        if f.read().strip() == "running":
            return
    with open(PRU2_0 + "/firmware", "w") as f:
        f.write("am57xx-pru2_0-fw")
    with open(PRU2_0 + "/state", "w") as f:
        f.write("start")
    for _ in range(50):
        try:
            open("/dev/rpmsg_pru32").close()
            return
        except IOError:
            time.sleep(0.1)
    rospy.logwarn("/dev/rpmsg_pru32 did not appear; motor 1 may only turn one way")


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BaseNode(object):
    def __init__(self):
        p = rospy.get_param
        # Each side averages its listed encoder channels; signs make forward positive
        self.left_chs = list(p("~left_encoders", [1]))
        self.right_chs = list(p("~right_encoders", [2]))
        self.signs = {int(k): v for k, v in p("~encoder_signs", {}).items()}
        self.counts_per_rev = float(p("~counts_per_rev", 16.0))
        self.wheel_radius = float(p("~wheel_radius", 0.0325))
        self.track = float(p("~track_width", 0.13))
        self.use_gyro_heading = p("~use_gyro_heading", True)
        self.publish_tf = p("~publish_tf", True)
        self.rate_hz = float(p("~rate", 30.0))
        self.odom_frame = p("~odom_frame", "odom")
        self.base_frame = p("~base_frame", "base_link")
        self.imu_frame = p("~imu_frame", "imu_link")

        self.m_per_count = 2.0 * math.pi * self.wheel_radius / self.counts_per_rev

        # Motors: each side drives its listed cape channels; signs make forward positive
        self.left_motors = list(p("~left_motors", []))
        self.right_motors = list(p("~right_motors", []))
        self.motor_signs = {int(k): v for k, v in p("~motor_signs", {}).items()}
        self.max_wheel_speed = float(p("~max_wheel_speed", 0.5))  # m/s at duty 1.0
        self.max_duty = float(p("~max_duty", 0.6))
        self.min_duty = float(p("~min_duty", 0.0))  # overcome motor deadband
        self.cmd_timeout = float(p("~cmd_timeout", 0.5))
        self.motor = None
        if self.left_motors or self.right_motors:
            start_motor_dir_pru()
            import rcpy.motor as motor  # runs rc_motor_init
            self.motor = motor
            self.stop_motors()
        self.last_cmd_time = None

        mpu9250.initialize(enable_magnetometer=True)
        self.gyro_bias_z = self.measure_gyro_bias()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
        self.mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size=10)
        self.tick_pub = rospy.Publisher("wheel_ticks", Int32MultiArray, queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        if self.motor:
            rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel, queue_size=1)

        self.x = self.y = self.yaw = 0.0
        self.last_l, self.last_r = self.read_ticks()
        self.last_t = rospy.Time.now()

    def set_side(self, channels, speed):
        duty = speed / self.max_wheel_speed
        if abs(duty) > 1e-3:
            duty = math.copysign(self.min_duty + (1.0 - self.min_duty) * abs(duty), duty)
        else:
            duty = 0.0
        duty = max(-self.max_duty, min(self.max_duty, duty))
        for ch in channels:
            self.motor.set(ch, self.motor_signs.get(ch, 1) * duty)

    def stop_motors(self):
        for ch in self.left_motors + self.right_motors:
            self.motor.set(ch, 0.0)

    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        self.set_side(self.left_motors, v - w * self.track / 2.0)
        self.set_side(self.right_motors, v + w * self.track / 2.0)
        self.last_cmd_time = time.time()

    def side_ticks(self, chs):
        return sum(self.signs.get(ch, 1) * encoder.get(ch) for ch in chs) / float(len(chs))

    def read_ticks(self):
        return self.side_ticks(self.left_chs), self.side_ticks(self.right_chs)

    def measure_gyro_bias(self, seconds=2.0):
        rospy.loginfo("Measuring gyro bias, keep the robot still...")
        samples = []
        end = time.time() + seconds
        while time.time() < end and not rospy.is_shutdown():
            samples.append(mpu9250.read_gyro_data()[2])
            time.sleep(0.01)
        bias = sum(samples) / max(len(samples), 1)
        rospy.loginfo("Gyro z bias: %.3f deg/s", bias)
        return bias

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_t).to_sec()
        if dt <= 0.0:
            return
        self.last_t = now

        data = mpu9250.read()
        ax, ay, az = data["accel"]
        gx, gy, gz = [g * DEG2RAD for g in data["gyro"]]
        gz_corr = gz - self.gyro_bias_z * DEG2RAD

        l, r = self.read_ticks()
        dl = (l - self.last_l) * self.m_per_count
        dr = (r - self.last_r) * self.m_per_count
        self.last_l, self.last_r = l, r

        d = (dl + dr) / 2.0
        if self.use_gyro_heading:
            dyaw = gz_corr * dt
        else:
            dyaw = (dr - dl) / self.track
        self.x += d * math.cos(self.yaw + dyaw / 2.0)
        self.y += d * math.sin(self.yaw + dyaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        q = yaw_to_quat(self.yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = d / dt
        odom.twist.twist.angular.z = dyaw / dt
        for i, v in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            odom.pose.covariance[i * 7] = v
            odom.twist.covariance[i * 7] = v
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_pub.sendTransform(t)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_frame
        imu.orientation_covariance[0] = -1.0  # no orientation estimate
        imu.angular_velocity = Vector3(gx, gy, gz_corr)
        imu.linear_acceleration = Vector3(ax, ay, az)
        self.imu_pub.publish(imu)

        mx, my, mz = data.get("mag", (0.0, 0.0, 0.0))
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field = Vector3(mx * 1e-6, my * 1e-6, mz * 1e-6)  # uT -> T
        self.mag_pub.publish(mag)

        # raw counts of all four cape channels, for debugging wiring
        self.tick_pub.publish(Int32MultiArray(data=[encoder.get(ch) for ch in (1, 2, 3, 4)]))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if (self.motor and self.last_cmd_time is not None
                        and time.time() - self.last_cmd_time > self.cmd_timeout):
                    self.stop_motors()
                    self.last_cmd_time = None
                self.step()
                rate.sleep()
        finally:
            if self.motor:
                self.stop_motors()


if __name__ == "__main__":
    rospy.init_node("bbai_base")
    try:
        BaseNode().spin()
    finally:
        rcpy.exit()
