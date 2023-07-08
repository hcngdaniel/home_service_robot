#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
import typing
import numpy as np
from tf.transformations import euler_from_quaternion
import tf


class Chassis:
    def __init__(self, cmd_vel_topic="/cmd_vel", imu_topic = "/imu/onboard_imu",
                 odom_frame = "/odom", base_frame = '/base_link') -> None:
        self.cmd_vel_topic: typing.AnyStr = cmd_vel_topic
        self.imu_topic: typing.AnyStr = imu_topic
        rospy.Subscriber(self.imu_topic, Imu, callback=self.__imu_callback)
        self.__cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.imu_data: typing.Union[None, Imu] = None
        self.tf_listener = tf.TransformListener()
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))

    def __imu_callback(self, data):
        self.imu_data = data

    def get_odom(self):
        trans, rot = self.tf_listener.lookupTransform(
            self.odom_frame,
            self.base_frame,
            rospy.Time(0),
        )
        return Point(*trans), euler_from_quaternion(rot)

    def move(self, forward_speed: float = 0, turn_speed: float = 0):
        msg = Twist()
        msg.linear.x = forward_speed
        msg.angular.z = turn_speed
        self.__cmd_vel_pub.publish(msg)

    def turn_to(self, angle: float, speed: float):
        max_speed = 1.82
        limit_time = 8
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            _, q = self.get_odom()
            roll, pitch, yaw = q
            e = angle - yaw
            if yaw < 0 and angle > 0:
                cw = np.pi + yaw + np.pi - angle
                aw = -yaw + angle
                if cw < aw:
                    e = -cw
            elif yaw > 0 and angle < 0:
                cw = yaw - angle
                aw = np.pi - yaw + np.pi + angle
                if aw < cw:
                    e = aw
            if abs(e) < 0.01 or rospy.get_time() - start_time > limit_time:
                break
            self.move(0.0, max_speed * speed * e)
            rospy.Rate(20).sleep()
        self.move(0.0, 0.0)

    def turn(self, angle: float, speed: float):
        _, q = self.get_odom()
        roll, pitch, yaw = q
        target = yaw + angle
        if target > np.pi:
            target = target - np.pi * 2
        elif target < -np.pi:
            target = target + np.pi * 2
        self.turn_to(target, speed)

    def move_for(self, distance, speed):
        start_pos, start_angle = self.get_odom()
        now_pos, now_angle = start_pos, start_angle
        while ((start_pos.x - now_pos.x) ** 2 + (start_pos.y - now_pos.y) ** 2) ** 0.5 < distance and not rospy.is_shutdown():
            self.move(speed, 0)
            now_pos, now_angle = self.get_odom()
            rospy.Rate(16).sleep()
        self.move(0, 0)
