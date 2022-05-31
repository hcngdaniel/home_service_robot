#!/usr/bin/env python3
import rospy
import core
import time
import cv2
import cv_bridge
from sensor_msgs.msg import Image


rospy.init_node("manipulator_demo2")
rospy.loginfo("manipulator_demo2 node started!")

astra = core.Astra(compressed=False)
# manipulator = core.Manipulator()
solution = core.neural_networks.Solution()
yolo = core.neural_networks.Yolov5()
faster_rcnn = core.neural_networks.FasterRCNN()
respeaker = core.Respeaker()
nlu = core.nlu.Assistant()
dataset = core.nlu.Dataset().from_yaml("manipulator_demo2_nlu.yaml")
nlu.set_dataset(dataset)
hccd = core.neural_networks.HCCD()
pub = rospy.Publisher("/imgstream", Image, queue_size=10)

time.sleep(1)

home_pose = (0.135, 0.0, 0.235)
init_pose = (0.287, 0.0, 0.235)

# while not rospy.is_shutdown():
#     text = respeaker.get_text()
#     text = text.lower()
#     intent = nlu.session.request(text)
#     if intent.intent_name == "help":
#         break
#     rospy.Rate(20).sleep()
# respeaker.say("ok")
# manipulator.move_to(*init_pose, 3, slices=1)
# manipulator.open_gripper(1)
# time.sleep(2)
# manipulator.close_gripper(1)
# manipulator.move_to(*home_pose, 3, slices=1)
# time.sleep(2)
# while not rospy.is_shutdown():
#     text = respeaker.get_text()
#     text = text.lower()
#     intent = nlu.session.request(text)
#     if intent.intent_name == "thank":
#         break
#     rospy.Rate(20).sleep()
# respeaker.say("you are welcome")

COLOR = [
    (0, 0, 255),
    (0, 255, 255),
    (0, 255, 0)
]

while not rospy.is_shutdown():
    # text = respeaker.get_text()
    # text = text.lower()
    # intent = nlu.session.request(text)
    # if intent.intent_name == "open":
    #     manipulator.open_gripper(1)
    rgb = astra.read_rgb()
    depth = astra.read_depth()
    results = solution.process(rgb, [faster_rcnn])
    print(results)
    if results:
        boxes = results[faster_rcnn.name].boxes
        for box in boxes:
            if box.conf[0] >= 0.8:
                cv2.rectangle(rgb, (box.left, box.top), (box.right, box.bottom), (0, 255, 0), 2)
                cv2.putText(rgb, box.class_name, (box.left, box.top), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("rgb", rgb)
    cv2.waitKey(1)
    pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(rgb, "bgr8"))
    rospy.Rate(20).sleep()
