#!/usr/bin/env python3
import rospy
import cv2
import core
from hccd.api import detect_code
from Book_Recommendation.api import book_recommendation
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import pyttsx3


rospy.init_node("live_demo")
astra = core.Astra()

person_count = 0
pc_frame = np.zeros((480, 640, 3), dtype="uint8")


def person_count_cb(msg):
    global person_count
    person_count = int(msg.data)


def pc_frame_cb(msg):
    global pc_frame
    pc_frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")


person_count_sub = rospy.Subscriber("/personcnt", Int32, callback=person_count_cb)
pc_imgstream_sub = rospy.Subscriber("/frame_for_person_count", Image, callback=pc_frame_cb)

start_time = time.time() + 99999999
flag = True
while not rospy.is_shutdown():
    cv2.imshow("frame", pc_frame)
    key = cv2.waitKey(16)
    if key in [27, ord('q')]:
        break
    if person_count == 1 and flag:
        start_time = time.time()
        flag = False
    if time.time() - start_time > 0.5:
        break
pyttsx3.speak("hello, please show me your health code")

while not rospy.is_shutdown():
    frame = astra.read_rgb()
    code_frame, color = detect_code(frame)
    cv2.imshow("frame", code_frame)
    key = cv2.waitKey(16)
    if key in [27, ord('q')]:
        break
    if color is not None:
        if color == "green":
            break

pyttsx3.speak("your health code is green, you may pass")
