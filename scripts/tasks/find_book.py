#!/usr/bin/env python3
import sys
import rospkg
import rospy
try:
    sys.path.append(f"{rospkg.RosPack().get_path('home_service_robot')}/scripts")
except:
    sys.path.append("/home/hcng/catkin_ws/src/home_service_robot/scripts")

import core


astra = core.Astra()
navigation = core.Navigation()

