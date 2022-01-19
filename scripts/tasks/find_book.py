#!/usr/bin/env python3
import sys
import rospkg
import rospy
from geometry_msgs.msg import Pose
try:
    sys.path.append(f"{rospkg.RosPack().get_path('home_service_robot')}/scripts")
except:
    sys.path.append("/home/hcng/catkin_ws/src/home_service_robot/scripts")

import core


rospy.init_node('test')
assistant = core.nlu.Assistant()
# navigation = core.Navigation()
pose_dict = core.utils.PoseDict.load("test.yaml")
del pose_dict["a"]
print(pose_dict)
exit()
# pose = navigation.point_to_pose(1.13791871071, -0.632079899311, 0.403900146484)
# navigation.move_to(pose.position.x, pose.position.y, pose.orientation.z)
while not rospy.is_shutdown():
    pose = navigation.get_clicked_pose_in_rviz()
    print(navigation.move_to(pose.position.x, pose.position.y, pose.orientation.z))
