#!/usr/bin/env python3
import rospy
import core
import numpy as np
import time


rospy.init_node('asdf')
kobuki = core.Kobuki()
time.sleep(2)
print('hihi')
kobuki.turn(np.deg2rad(45))
