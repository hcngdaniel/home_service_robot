#!/usr/bin/env python3
import core
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image


rospy.init_node("hccd_demo")
rospy.loginfo("hccd_demo node started")

cap = cv2.VideoCapture(0)
solution = core.neural_networks.Solution()
hccd = core.neural_networks.HCCD()
pub = rospy.Publisher("/hccd_demo", Image, queue_size=10)

GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
RED = (0, 0, 255)

while not rospy.is_shutdown():
    if not cap.isOpened():
        print("no cam")
        continue
    success, frame = cap.read()
    if not success:
        print("can't read")
        continue
    results = solution.process(frame, [hccd])
    if results:
        boxes = results[hccd.name]
        for box in boxes.boxes:
            color = None
            if box.class_name == "green":
                color = GREEN
            if box.class_name == "yellow":
                color = YELLOW
            if box.class_name == "red":
                color = RED
            cv2.rectangle(frame, (box.left, box.top), (box.right, box.bottom), color, 2)
    print(frame.shape)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(16)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
