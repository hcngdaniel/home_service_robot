#!/usr/bin/env python3
import rospy

from neural_networks.msg import BoundingBoxes
from .base_solution import BaseSolution


class Yolov5(BaseSolution):

    def __init__(self):
        super(Yolov5, self).__init__()
        self.name = 'yolov5'
        self.result_topic = '/neural_networks/results/yolov5'
        rospy.Subscriber(self.result_topic, BoundingBoxes, callback=self.__callback__)

    def __callback__(self, msg):
        self.result = msg

