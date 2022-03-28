#!/usr/bin/env python3
import rospy
import core


rospy.init_node("turtlebot_demo")
kobuki = core.Kobuki()
kobuki.cmd_vel_topic = "/mobile_base/commands/velocity"
kobuki.imu_topic = "/mobile_base/sensors/imu_data_raw"
rospy.Rate(1).sleep()

# while not rospy.is_shutdown():
for i in range(1000000):
    kobuki.move(0.05, 0)
    rospy.Rate(100).sleep()
# rospy.Rate(1).sleep()
# kobuki.move(0, 0)

