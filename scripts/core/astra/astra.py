#!usr/bin/env python3
import os
import sys
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import typing
from .. import utils


class Astra:
    def __init__(self, namespace: str = "/camera", compressed = False) -> None:
        self.ns: str = namespace
        self.compressed = compressed
        self.__bgr_img = None
        self.__depth_img = None
        rospy.Subscriber(self.ns + "/depth/image_raw", Image, callback=self.__depth_img_callback)
        if not compressed:
            rospy.Subscriber(self.ns + "/rgb/image_raw", Image, callback=self.__rgb_img_callback)
        else:
            rospy.Subscriber(self.ns + "/rgb/image_raw/compressed", CompressedImage, callback=self.__rgb_img_callback)

    def __rgb_img_callback(self, msg) -> None:
        if not self.compressed:
            img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.__bgr_img = img
        else:
            img = np.fromstring(msg.data, np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            self.__bgr_img = img
    
    def __depth_img_callback(self, msg) -> None:
        if not self.compressed:
            img = CvBridge().imgmsg_to_cv2(msg, "passthrough")
            self.__depth_img = img
    
    def read_rgb(self):
        if self.__bgr_img is not None:
            return self.__bgr_img
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def read_depth(self):
        if self.__depth_img is not None:
            return self.__depth_img
        return np.zeros((480, 640), dtype=np.uint8)

    @staticmethod
    def get_real_xyz(x, y, depth_img) -> typing.Tuple[float, float, float]:
        depth = depth_img[y][x]
        horiz_len = 2 * np.tan(np.deg2rad(30)) * depth
        vert_len = 2 * np.tan(np.deg2rad(24.75)) * depth
        rz = float(depth)
        rx = float((x - 320) / 640 * horiz_len)
        ry = float((y - 240) / 480 * vert_len)
        return rx, ry, rz

    def get_euclidean_distance(self, x, y, depth_img):
        rx, ry, rz = self.get_real_xyz(x, y, depth_img)
        distance = utils.distance((rx, ry, rz), (0, 0, 0))
        return distance
    
    def __call__(self):
        return {"rgb": self.read_rgb,
                "depth": self.read_depth}
    
    def __str__(self) -> str:
        return self.read_rgb() + "\n" + self.read_depth

