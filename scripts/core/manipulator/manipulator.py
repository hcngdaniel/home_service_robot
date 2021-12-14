#!/usr/bin/env python3
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
import time


class Manipulator:
    def __init__(self):
        self.service_name = "/goal_task_space_path_position_only"
    
    def move_directly_to(self, x, y, z, t):
        rospy.wait_for_service(self.service_name)
        service_proxy = rospy.ServiceProxy(self.service_name, SetKinematicsPose)
        
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.path_time = t
        
        response = service_proxy(request)
        return response

