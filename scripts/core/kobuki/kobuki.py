#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import typing
import numpy as np
from tf.transformations import euler_from_quaternion


class Kobuki:
    def __init__(self) -> None:
        self.cmd_vel_topic: typing.AnyStr = '/mobile_base/commands/velocity'
        self.imu_topic: typing.AnyStr = '/mobile_base/sensors/imu_data'
        rospy.Subscriber(self.imu_topic, Imu, callback=self.__imu_callback)
        self.__cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist)
        self.imu_data: typing.Union[None, Imu] = None

    def __imu_callback(self, data):
        self.imu_data = data

    def move(self, forward_speed: float = 0, turn_speed: float = 0):
        msg = Twist()
        msg.linear.x = forward_speed
        msg.angular.z = turn_speed
        self.__cmd_vel_pub.publish(msg)

    def turn_to(self, angle: float, speed: float):
        max_speed = 1.82
        limit_time = 8
        start_time = rospy.get_time()
        while True:
            q = [
                self.imu_data.orientation.x,
                self.imu_data.orientation.y,
                self.imu_data.orientation.z,
                self.imu_data.orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(q)
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

    def turn(self, angle: float):
        q = [
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)
        target = yaw + angle
        if target > np.pi:
            target = target - np.pi * 2
        elif target < -np.pi:
            target = target + np.pi * 2
        self.turn_to(target, 0.5)
