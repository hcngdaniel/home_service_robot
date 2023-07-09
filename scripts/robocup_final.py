#!/usr/bin/env python3
import core
import rospy
import cv2
import numpy as np
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from easydict import EasyDict as edict


def state_callback(msg):
    global door_state
    door_state = msg.data

def filter_detections(dets, cls):
    res = []
    for det in dets:
        if yolo.classes[int(det[5])] == cls:
            res.append(det)
    return res

def filter_dis(dets, depth, threshold):
    res = []
    for det in dets:
        cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
        dis = up_cam.get_euclidean_distance(cx, cy, depth)
        if dis < threshold:
            res.append(det)
    return res

def open():
    global pub_cmd
    cmd = Twist()
    cmd.angular.z = math.pi / 2
    pub_cmd.publish(cmd)

def close():
    global pub_cmd
    cmd = Twist()
    cmd.angular.z = -math.pi / 2
    pub_cmd.publish(cmd)

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
respeaker = core.Respeaker()
yolo = core.openvino_models.Yolov8("yolov8n")
chassis = core.Chassis()
rate = rospy.Rate(30)
assistant = core.nlu.Assistant()
assistant.set_dataset(core.nlu.Dataset().from_yaml("robocup_final.yaml"))
session = assistant.session
navigation = core.Navigation()
manipulator = core.Manipulator()
state_pub = rospy.Publisher("/robot_states", String, queue_size=3)
state_sub = rospy.Subscriber("/door_states", String, callback=state_callback)
pub_cmd = rospy.Publisher("/door_lock", Twist, queue_size=10)

poses = edict()
poses.center = (0.0, 0.0, 0.0)
poses.door = (0.0, 0.0, 0.0)

home_pose = (0.000, -1.049, 0.357, 0.703)
init_pose = (0.000, 0.012, 0.035, 0.009)
drop_pose = (0.000, 0.012, 0.035, 1.500)


manipulator.set_joint(*init_pose, 1.5)
manipulator.set_joint(*home_pose, 1.5)
manipulator.close_gripper(1.5)

respeaker.say("I am ready")

while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()

    detections = yolo.forward(frame)[0]['det']
    yolo.draw_bounding_box({'det': detections}, frame)
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 3500)

    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    for det in detections:
        w, h = det[2] - det[0], det[3] - det[1]
        if w / h > 2:
            break
    
    rate.sleep()

print("event: person pk")

respeaker.say("Do you need help?")

time.sleep(8)

respeaker.say("Do you need me to call the ambulance?")

time.sleep(8)

# alarm sound

state_pub.publish("person pk")

while not rospy.is_shutdown():
    if door_state == "person arrived":
        break
    rate.sleep()

respeaker.say("Hello, I will open the door lock")

open()

respeaker.say("please push the door and save my master")

time.sleep(10)

respeaker.say("do not forget to close the door")

time.sleep(10)

close()

state_pub.publish("first person ready to go")

while not rospy.is_shutdown():
    if door_state == "errand1 arrived":
        break
    rate.sleep()

navigation.move_to(*poses.door)

respeaker.say("I don't know who are you. I won't open the door for you.")

navigation.move_to(*poses.center)

state_pub.publish("second person ready to go")

while not rospy.is_shutdown():
    if door_state == "errand2 arrived":
        break
    rate.sleep()

navigation.move_to(*poses.door)

respeaker.say("hello, daniel I will open the door lock for you because my master told me that you will bring something here")

open()

respeaker.say("please push the door and come in")

time.sleep(5)

manipulator.set_joint(*init_pose, 1.5)

respeaker.say("please put the things on my hand")

time.sleep(5)

manipulator.set_joint(*home_pose, 1.5)

respeaker.say("bye. please do not forget to close the door")

time.sleep(5)

close()

navigation.move_to(*poses.center)

manipulator.set_joint(*drop_pose, 1.5)
