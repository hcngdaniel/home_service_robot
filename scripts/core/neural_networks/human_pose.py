#!/usr/bin/env python3
import rospy
from neural_networks.msg import HumanPose as HP
from .base_solution import BaseSolution


class HumanPose(BaseSolution):
    def __init__(self):
        super(HumanPose, self).__init__()
        self.name = "human_pose"
        self.result_topic = "/neural_networks/results/human_pose"
        rospy.Subscriber(self.result_topic, HP, callback=self.__callback__)

    def __callback__(self, msg):
        self.result = msg
