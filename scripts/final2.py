#!/usr/bin/env python4
import rospy
import core
import time
import numpy as np
import cv2
from PIL import Image


rospy.init_node("final")

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
respeaker = core.Respeaker()
yolo = core.openvino_models.Yolov8("yolov8n")
# chassis = core.Chassis()
rate = rospy.Rate(30)
assistant = core.nlu.Assistant()
assistant.set_dataset(core.nlu.Dataset().from_yaml("final2_dataset.yaml"))
session = assistant.session
# navigation = core.Navigation()

door_pos = (0.0, 0.0, 0.0)


def filter_class(dets, class_name):
    res = []
    for det in dets:
        if yolo.classes[int(det[5])] == class_name:
            res.append(det)
    return res

def filter_conf(dets, threshold):
    res = []
    for det in dets:
        if det[4] > threshold:
            res.append(det)
    return res

def filter_dis(dets, depth, threshold = 2000):
    res = []
    for det in dets:
        cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
        dis = up_cam.get_euclidean_distance(cx, cy, depth)
        if dis < threshold:
            res.append(det)
    
    return res


def open_door():
    pass

def close_door():
    pass

# start
respeaker.say("I am ready")

target = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()

    detections = yolo.forward(frame)
    show_frame = frame.copy()
    yolo.draw_bounding_box(detections[0], show_frame)
    detections = detections[0]['det']
    cv2.imshow("frame", show_frame)
    cv2.waitKey(1)

    persons = filter_class(detections, "person")
    persons = filter_conf(persons, 0.8)
    persons = filter_dis(persons, depth, 2000)

    targets = []
    for person in persons:
        dx, dy = person[2] - person[0], person[3] - person[1]
        if dx / dy > 2:
            targets.append(person)
    if targets:
        target = targets[0]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break

    rate.sleep()

cx, cy = int(target[0] + target[2]) // 2, int(target[1] + target[3]) // 2
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)
chassis.turn(angle, np.radians(20))

print("event: person falled down")
respeaker.say("Hello, are you alright?")

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "CallAmbulance":
            break

    rate.sleep()

print("event: call ambulance")

# alarm sound

navigation.move_to(*door_pos)

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "OpenDoor":
            break

respeaker.say("Okay, I will open the door")

open_door()

respeaker.say("You may now push the door and come in")

time.sleep(10)

respeaker.say("Don't worry, I will help you to look after your home")


