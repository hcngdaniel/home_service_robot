#!/usr/bin/env python3
import os
os.chdir(os.path.dirname(__file__))
import rospy
from std_msgs.msg import Bool
import cv2


rospy.init_node("button_node")

rospy.loginfo("node started!")

button_img = cv2.imread("button.jpg")
msg_pub = rospy.Publisher("/button_state", Bool, queue_size=10)

while not rospy.is_shutdown():
    cv2.imshow("button", button_img)
    key = cv2.waitKey(64)
    if key == ord(' '):
        msg_pub.publish(True)
    else:
        msg_pub.publish(False)
    if key == 27:
        break

cv2.destroyAllWindows()
