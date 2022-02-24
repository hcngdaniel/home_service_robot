#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import uuid

from sensor_msgs.msg import Image

from .hccd import HCCD


class Solution:

    model_names = [
        'hccd',
    ]

    def __init__(self):
        self.img_pub = rospy.Publisher('neural_networks/image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.models = {
            'hccd': HCCD()
        }

    def process(self, img, model_names, img_encoding = 'bgr8', rate: rospy.Rate = None):
        if rate is None:
            rate = rospy.Rate(20)
        imgmsg = self.bridge.cv2_to_imgmsg(img, img_encoding)
        imgmsg.header.frame_id = ' '.join(model_names + [str(uuid.uuid4())])
        self.img_pub.publish(imgmsg)
        results_dict = {key: None for key in model_names}
        while not rospy.is_shutdown():
            rate.sleep()
            for name, model in self.models.items():
                if model.result is not None:
                    if model.result.header.frame_id == imgmsg.header.frame_id:
                        results_dict[name] = model.result
            if None not in results_dict.values():
                break
        print(results_dict)
        return results_dict
