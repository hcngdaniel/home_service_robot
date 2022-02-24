#!/usr/bin/env python3
import rospy

from neural_networks.msg import BoundingBoxes
from .base_solution import BaseSolution


class HCCD(BaseSolution):
    def __init__(self):
        super(HCCD, self).__init__()
        self.result_topic = '/neural_networks/results/hccd'
        rospy.Subscriber(self.result_topic, BoundingBoxes, callback=self.__callback__)

    def __callback__(self, msg):
        self.result = msg
