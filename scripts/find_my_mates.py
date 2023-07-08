#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import core
import rospy
import time
import numpy as np
import cv2
from collections import deque
from PIL import Image
from namelist import name_list


MAX_DIST = 2500
MAX_DTHETA = 5
TURN_SPEED = np.radians(10)
debug = 'mf'

rospy.init_node("find_my_mates")

up_cam = core.Astra("ucam")
# down_cam = core.Astra("dcam")
chassis = core.Chassis()
respeaker = core.Respeaker()
dataset = core.nlu.Dataset().from_yaml("fmm_dataset.yaml")
assistant = core.nlu.Assistant()
assistant.set_dataset(dataset)
# assistant = core.nlu.Assistant.load("fmm_assistant.ass")
session = assistant.session
navigation = core.Navigation()
# manipulator = core.Manipulator()
yolo = core.openvino_models.Yolov8("yolov8n")
PA100k_PAR = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR")
PETA_PAR = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR_PETA")

time.sleep(3)

def filter_detections(dets):
    for det in dets:
        if det[5] == 0:
            yield det

def filter_person(persons, depth, max_depth = MAX_DIST): # star
    out = []
    for person in persons:
        cx, cy = int(person[0] + person[2]) // 2, int(person[1] + person[3]) // 2
        rx, ry, rz = up_cam.get_real_xyz(cx, cy, depth, autofill=False)
        print((rx ** 2 + rz ** 2), end=' ')
        if 0 < (rx ** 2 + rz ** 2) ** 0.5 <= max_depth:
            out.append((cx, cy, rz, person))
    print("")
    return out

start_point = (0, 0, 3.14)
mid_point = (1.19, -4.45, 0)

# navigation.move_to(*start_point)

respeaker.say("I am ready")

while not rospy.is_shutdown():
    text = respeaker.get_text()
    # text = input()
    if text:
        print(text)
        session.request(text)
        if session.intent_name == "findmates":
            break
    rospy.Rate(16).sleep()

print("event: find mates")
respeaker.say(f"okay, I will find your mates")

navigation.move_to(*mid_point)

print("event: arrived mid point")

person1 = None
frame = None
while not rospy.is_shutdown():
    chassis.move(0, TURN_SPEED)
    frame = up_cam.read_rgb()
    # print(frame)
    
    dets = yolo.forward(frame)[0]['det']
    persons = filter_detections(dets)
    depth = up_cam.read_depth()
    near_persons = filter_person(persons, depth)

    print(list(near_persons))
    max_offset = 40
    
    for person in near_persons:
        print(person[0], end=" ")
        if 320 - max_offset <= person[0] <= 320 + max_offset:
            person1 = person
    print("")

    if person1 is not None:
        break
    
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(16).sleep()

chassis.move(0, 0)

yolo.draw_bounding_box({"det": [person1[3]]}, frame)
img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
img.show()
img = frame.copy()

chassis.turn(np.radians(-5), TURN_SPEED)
chassis.move_for((person1[2] - 1000) / 1000, 0.2)

respeaker.say("what is your name")

person1_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "mynameis":
            if "name" not in session.missing_slots:
                person1_name = session.get_slot_value("name")
                if person1_name.lower() in name_list:
                    break
    rospy.Rate(16).sleep()

print(f"event: recieved name: {person1_name}")

navigation.move_to(*mid_point)
time.sleep(0.2)
navigation.move_to(*start_point)

out_txt = f"The first guest I saw is named {person1_name}"
if debug[0] == "m":
    out_txt += ", who is wearing shorts and short sleeves clothe"
elif debug[0] == 'f':
    out_txt += ", who is a female and have long hair"
out_txt += ", and standing besides the long desk"
respeaker.say(out_txt)
print("spoken text: ", end=' ')
print(out_txt)

while not rospy.is_shutdown():
    text = respeaker.get_text()
    # text = input()
    if text:
        print(text)
        session.request(text)
        if session.intent_name == "findmates":
            break
    rospy.Rate(16).sleep()

print("event: find mates")
respeaker.say(f"okay, I will find your mates")

navigation.move_to(*mid_point)

print("event: arrived mid point")

person2 = None
frame = None
while not rospy.is_shutdown():
    chassis.move(0, -TURN_SPEED)
    frame = up_cam.read_rgb()
    # print(frame)
    
    dets = yolo.forward(frame)[0]['det']
    persons = filter_detections(dets)
    depth = up_cam.read_depth()
    near_persons = filter_person(persons, depth)

    print(list(near_persons))
    max_offset = 40
    
    for person in near_persons:
        print(person[0], end=" ")
        if 320 - max_offset <= person[0] <= 320 + max_offset:
            person2 = person
    print("")

    if person2 is not None:
        break
    
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(16).sleep()

chassis.move(0, 0)

yolo.draw_bounding_box({"det": [person2[3]]}, frame)
img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
img.show()
img = frame.copy()

chassis.turn(np.radians(5), TURN_SPEED)
chassis.move_for((person2[2] - 1000) / 1000, 0.2)

respeaker.say("what is your name")

person2_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "mynameis":
            if "name" not in session.missing_slots:
                person2_name = session.get_slot_value("name")
                if person2_name.lower() in name_list:
                    break
    rospy.Rate(16).sleep()

print(f"event: recieved name: {person2_name}")

navigation.move_to(*mid_point)
time.sleep(0.2)
navigation.move_to(*start_point)

print("spoken text: ", end=' ')
out_txt = f"There is a mate named {person2_name}"
if debug[1] == "m":
    out_txt += ", who is wearing shorts and short sleeves clothe"
elif debug[1] == 'f':
    out_txt += ", who is a female and have long hair"
if navigation.pose_to_point(navigation.current_pose.pose.pose)[2] > 1.7:
    out_txt += ", and standing besides the drawers"
elif navigation.pose_to_point(navigation.current_pose.pose.pose)[2] < 1.4:
    out_txt += ", and standing besides the bins"
else:
    out_txt += ", and standing besides the long desk"
respeaker.say(out_txt)
print(out_txt)

