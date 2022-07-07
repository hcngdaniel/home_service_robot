#!/usr/bin/env python3
import rospy
import time
from cv_bridge import CvBridge
import uuid
import warnings

from sensor_msgs.msg import Image


class Solution:

    MODEL_NAMES = [
        'hccd',
        'yolov5',
        'fasterrcnn',
        "human_pose"
    ]

    def __init__(self):
        self.img_pub = rospy.Publisher('/neural_networks/image', Image, queue_size=10)
        self.bridge = CvBridge()

    def process(self, img, models, img_encoding = 'bgr8', rate: rospy.Rate = None):
        if rate is None:
            rate = rospy.Rate(20)
        model_names = [model.name for model in models]
        imgmsg = self.bridge.cv2_to_imgmsg(img, img_encoding)
        imgmsg.header.frame_id = ' '.join(model_names + [str(uuid.uuid4())])
        self.img_pub.publish(imgmsg)
        results_dict = {key: None for key in model_names}

        start_time = time.time()
        while not rospy.is_shutdown():
            rate.sleep()
            for model in models:
                if model.result is not None:
                    if model.result.header.frame_id == imgmsg.header.frame_id:
                        results_dict[model.name] = model.result
            if None not in results_dict.values():
                break
            if time.time() - start_time > 1.0:
                warnings.warn("Request Timeout")
                return
        return results_dict
