#!/usr/bin/env python3
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.msg import KinematicsPose
import time


class Manipulator:
    def __init__(self, move_service="/goal_task_space_path_position_only", xyz_topic="/gripper/kinematics_pose"):
        self.move_service = move_service
        self.xyz_topic = xyz_topic
        rospy.Subscriber(self.xyz_topic, KinematicsPose, callback=self.__xyz_callback)
        first_msg = rospy.wait_for_message(self.xyz_topic, KinematicsPose)
        self.x = first_msg.pose.position.x
        self.y = first_msg.pose.position.y
        self.z = first_msg.pose.position.z
        
    def __xyz_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
    
    def move_directly_to(self, x, y, z, t, delay=0.05):
        rospy.wait_for_service(self.move_service)
        service_proxy = rospy.ServiceProxy(self.move_service, SetKinematicsPose)
        
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.path_time = t
        
        response = service_proxy(request)
        time.sleep(t + delay)
        return response
    
    def get_xyz(self):
        return (self.x, self.y, self.z)
        
    def move_to(self, x, y, z, t):
        t = round(t, 1)
        x1, y1, z1 = self.get_xyz()
        x2, y2, z2 = x, y, z
        if None in (x2, y2, z2):
            return False
        fy = lambda x: (y1 - y2) / (x1 - x2) * x + (y1 - (y1 - y2) / (x1 - x2) * x1)
        fz = lambda x: (z1 - z2) / (x1 - x2) * x + (z1 - (z1 - z2) / (x1 - x2) * x1)
        for i in range(int(t * 10)):
            goal_x = (x2 - x1) / (t * 10)
            goal_y, goal_z = fy(x), fz(x)
            if not (round(goal_x, 3) == round(x1, 3) and round(goal_y, 3) == round(y1, 3) and round(goal_z, 3) == round(z1, 3)):
                rospy.loginfo(self.move_directly_to(round(goal_x, 3), round(goal_y, 3), round(goal_z, 3), 0.1, delay=0.1))
        return True

