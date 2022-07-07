#!/usr/bin/env python3
import rospy
from .base_solution import BaseSolution
from neural_networks.msg import BoundingBoxes


class FasterRCNN(BaseSolution):
    def __init__(self):
        super(FasterRCNN, self).__init__()
        self.name = 'fasterrcnn'
        self.result_topic = '/neural_networks/results/fasterrcnn'
        rospy.Subscriber(self.result_topic, BoundingBoxes, callback=self.__callback__)

    def __callback__(self, msg):
        self.result = msg
