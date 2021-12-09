#!usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from numpy import zeros, uint8


class Astra:
    def __init__(self, topic: str = "/camera") -> None:
        self.topic: str = topic
        self.__bgr_img = None
        self.__depth_img = None
        rospy.Subscriber(self.topic + "/rgb/image_raw", Image, callback=self.__rgb_img_callback)
        rospy.Subscriber(self.topic + "/depth/image_raw", Image, callback=self.__depth_img_callback)

    def __rgb_img_callback(self, msg) -> None:
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.__bgr_img = img
    
    def __depth_img_callback(self, msg) -> None:
        img = CvBridge().imgmsg_to_cv2(msg, "passthrough")
        self.__depth_img = img
    
    def read_rgb(self):
        if self.__bgr_img is not None:
            return self.__bgr_img
        return zeros((480, 640, 3), dtype=uint8)
    
    def read_depth(self):
        if self.__depth_img is not None:
            return self.__depth_img
        return zeros((480, 640, 3), dtype=uint8)
    
    def __call__(self):
        return {"rgb": self.read_rgb,
                "depth": self.read_depth}
    
    def __str__(self) -> str:
        return self.read_rgb + "\n" + self.read_depth
