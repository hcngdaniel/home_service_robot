#!/usr/bin/env python3
import rospy
import core
import time
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("robot")
rospy.loginfo("robot node started!")
time.sleep(1)

top_cam = core.Astra("/top", compressed=True)
bottom_cam = core.Astra("/bottom", compressed=True)
manipulator = core.Manipulator()
# kobuki = core.Kobuki()
respeaker = core.Respeaker()
# navigation = core.Navigation()
rate = rospy.Rate(20)
time.sleep(1)
rospy.loginfo("hardwares loaded")

solution = core.neural_networks.Solution()
hccd = core.neural_networks.HCCD()
yolo = core.neural_networks.Yolov5()
# rcnn = core.neural_networks.FasterRCNN()
rospy.loginfo("neural networks loaded")

imgstream_pub = rospy.Publisher("/imgstream", Image, queue_size=10)

while not rospy.is_shutdown():
    top_frame = top_cam.read_rgb()
    results = solution.process(top_frame, [yolo], rate=rate)
    print(results)
    if not results: continue
    if results[yolo.name]:
        names = [i.class_name for i in results[yolo.name].boxes if i.conf[0] >= 0.8]
        if "person" in names: break
    rate.sleep()

respeaker.say("hello, please show me your health code")
rospy.loginfo("see a guy")

start_time = time.time()
flag = 0
while not rospy.is_shutdown():
    top_frame = top_cam.read_rgb()
    croped = top_frame[20:460, 100:540, :].copy()
    pub_msg = CvBridge().cv2_to_imgmsg(croped, "bgr8")
    imgstream_pub.publish(pub_msg)
    cv2.imshow("croped", croped)
    results = solution.process(top_frame, [hccd], rate=rate)
    cv2.waitKey(1)
    rate.sleep()
    if time.time() - start_time > 10:
        if not results: continue
        if results[hccd.name]:
            if "green" == results[hccd.name].data:
                flag = "pass"
            else:
                flag = "not pass"
            break

if flag == "pass":
    respeaker.say("your health code is green, you may pass")
else:
    respeaker.say("your health code isn't green, please see doctor as soon as possible")



# post progress
cv2.destroyAllWindows()
