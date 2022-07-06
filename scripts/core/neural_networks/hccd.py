#!/usr/bin/env python3
import rospy

from neural_networks.msg import StringWithHeader
from .base_solution import BaseSolution


class HCCD(BaseSolution):

    CLASSES = [
        'red',
        'yellow',
        'green'
    ]

    def __init__(self):
        super(HCCD, self).__init__()
        self.name = 'hccd'
        self.result_topic = '/neural_networks/results/hccd'
        rospy.Subscriber(self.result_topic, StringWithHeader, callback=self.__callback__)

    def __callback__(self, msg):
        self.result = msg
