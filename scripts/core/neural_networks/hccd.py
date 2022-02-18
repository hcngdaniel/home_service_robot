#!/usr/bin/env python3
import rospy
import typing
import numpy as np

from .base_solution import BaseSolution

from neural_networks.msg import BoundingBoxes


class HCCD(BaseSolution):
    def __init__(self):
        super(HCCD, self).__init__()
        rospy.Subscriber("neural_networks/results/hccd", BoundingBoxes, self.__result_callback)
        self.results = BoundingBoxes()

    def __result_callback(self, msg):
        self.results = msg

    def process(self, image: typing.Union[list, np.ndarray], rate: rospy.Rate = None):
        if rate is None:
            rate = rospy.Rate(20)
        self.send_image(image)
        while self.results.header.frame_id != self.expected_frame_id:
            rate.sleep()
        self.expected_frame_id = None
        return self.results.boxes
