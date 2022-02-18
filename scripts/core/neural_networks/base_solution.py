#!/usr/bin/env python3
from abc import ABC
import typing
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class BaseSolution(ABC):
    def __init__(self):
        self.expected_frame_id = None
        self.image_publisher = rospy.Publisher("neural_networks/image", Image, queue_size=1)

    def process(self, image: typing.Union[list, np.ndarray], rate: rospy.Rate = None):
        raise NotImplementedError()

    def __result_callback(self, msg):
        raise NotImplementedError()

    def send_image(self, image: typing.Union[list, np.ndarray]):
        imgmsg = CvBridge().cv2_to_imgmsg(image, "bgr8")
        self.expected_frame_id = imgmsg.header.frame_id
        self.image_publisher.publish(imgmsg)
