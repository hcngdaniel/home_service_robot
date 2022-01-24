#!/usr/bin/env python3
import sys
import os
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import typing


class Kobuki:
    def __init__(self) -> None:
        self.cmd_vel_topic: typing.AnyStr = '/mobile_base/commands/velocity'
        self.imu_topic: typing.AnyStr = '/mobile_base/sensors/imu_data'
        rospy.Subscriber(self.imu_topic, Imu, callback=self.__imu_callback)
        self.__cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist)
        self.imu_data: typing.Union[None, Imu] = None

    def __imu_callback(self, data):
        self.imu_data = data

    def turn(self, a):
        sys.path.append(f'{os.path.dirname(__file__)}/..')
        from utils import utils
        PID = utils.PID(1, 0, 0)
        del utils
        sys.path.pop()
        _, _, current_angle = euler_from_quaternion(self.imu_data)
        target = current_angle + a
        while not rospy.is_shutdown():
            _, _, current_angle = euler_from_quaternion(self.imu_data)
            msg = Twist()
            msg.linear.z = PID(target, current_angle)
            self.__cmd_vel_pub.publish(msg)
