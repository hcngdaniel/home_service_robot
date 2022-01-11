#!/usr/bin/env python3
import rospy
from core import *


rospy.init_node("asdf")
rospy.loginfo("Node started")
respeaker = Respeaker()
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text != "":
        print(repr(text))
    rospy.Rate(20).sleep()

