#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import core
import rospy
import time
import numpy as np
import cv2
from easydict import EasyDict as edict
from PIL import Image


rospy.init_node("find_my_mates")
print("node started")

up_cam = core.Astra("ucam")
chassis = core.Chassis()
respeaker = core.Respeaker()
dataset = core.nlu.Dataset().from_yaml("fmm_dataset.yaml")
manipulator = core.Manipulator()
assistant = core.nlu.Assistant()
assistant.set_dataset(dataset)
session = assistant.session
navigation = core.Navigation()
yolo = core.openvino_models.Yolov8("yolov8n")
PA100k = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR")
PETA = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR_PETA")
rate = rospy.Rate(30)

time.sleep(3)


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


poses = edict()
poses.master = (6.350534414321229, 1.8670363278890512, 2.239)
poses.cabinet = (2.814922236831155, -0.5162437382703509, 0.7855)
poses.wall_l = (2.164890237942357, 0.28803973953423073, -0.9825)
poses.wall_r = (2.175667028018338, 0.5788219233736409, -1.501)

home_pose = (0.000, -1.049, 0.357, 0.703)
init_pose = (0.000, 0.012, 0.035, 0.009)

manipulator.set_joint(*home_pose, 1.5)

# wait for command to go to living room
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "findmates":
            break
    rate.sleep()

print("event: find my mates")
respeaker.say("Okay, I will find your mates")

# navigate to cabinet
navigation.move_to(*poses.cabinet)

# detect person
target = None
person_img = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    dets = yolo.forward(frame)[0]['det']
    dets = filter_detections(dets, "person")
    dets = filter_dis(dets, depth, 4500)

    if dets:
        dets = sorted(dets, key=lambda det: det[0], reverse=True)
        target = dets[0]
        x1, y1, x2, y2, conf, cls = map(int, target)
        person_img = frame[y1:y2, x1:x2]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    rate.sleep()

print("event: detected person")

# approach the person
cx, cy = int(target[0] + target[2]) // 2, int(target[1] + target[3]) // 2
dis = up_cam.get_euclidean_distance(cx, cy, depth) / 1000
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)

chassis.turn(angle / abs(angle) * max(0, abs(angle) - 5), np.radians(20))
chassis.move_for(dis - 1, 0.2)

print("event: approached person")

# ask for the name
respeaker.say("what is your name")

person1_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["mynameis", "ENTITYname"] and "name" not in session.missing_slots:
            person1_name = session.get_slot_value("name")
            break
    
    rate.sleep()

print(f"event: received name: {person1_name}")
respeaker.say(f"nice to meet you, {person1_name}")

# process image
result = PA100k.forward(person_img)[0]
result = {k: v for k, v in zip(PA100k.classes, result)}
glasses = "wearing" if result['Glasses'] > 0.35 else "not wearing"
gender = "male" if result['Female'] < 0.4 else "female"

# go back to master
navigation.move_to(*poses.cabinet)
navigation.move_to(*poses.master)

# report
out_txt = f"the first mate i found is named {person1_name}, located besides the cabinet, and {glasses} glasses, its gender is {gender}"
respeaker.say(out_txt)
print(f"spoken text: {out_txt}")


# wait for command to go to living room
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "findmates":
            break
    rate.sleep()

print("event: find my mates")
respeaker.say("Okay, I will find your mates")

# navigate to fridge
navigation.move_to(*poses.wall_l)

# detect person
target = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    dets = yolo.forward(frame)[0]['det']
    dets = filter_detections(dets, "person")
    dets = filter_dis(dets, depth, 4500)

    if dets:
        dets = sorted(dets, key=lambda det: det[0])
        target = dets[0]
        x1, y1, x2, y2, conf, cls = map(int, target)
        person_img = frame[y1:y2, x1:x2]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    rate.sleep()

print("event: detected person")

# approach the person
cx, cy = int(target[0] + target[2]) // 2, int(target[1] + target[3]) // 2
dis = up_cam.get_euclidean_distance(cx, cy, depth) / 1000
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)

chassis.turn(angle / abs(angle) * max(0, abs(angle) - 5), np.radians(20))
chassis.move_for(dis - 1, 0.2)

print("event: approached person")

# ask for the name
respeaker.say("what is your name")

person2_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["mynameis", "ENTITYname"] and "name" not in session.missing_slots:
            person2_name = session.get_slot_value("name")
            break
    rate.sleep()

print(f"event: received name: {person2_name}")
respeaker.say(f"nice to meet you, {person2_name}")

# process image
result = PETA.forward(person_img)[0]
result = {k: v for k, v in zip(PETA.classes, result)}
upper_colors = sorted(['upperBodyBlack', 'upperBodyBlue', 'upperBodyBrown', 'upperBodyGreen', 'upperBodyGrey', 'upperBodyOrange', 'upperBodyPink', 'upperBodyPurple', 'upperBodyRed', 'upperBodyWhite', 'upperBodyYellow'],
                      key=lambda x: result[x],
                      reverse=True
                     )
down_colors = sorted(['lowerBodyBlack', 'lowerBodyBlue', 'lowerBodyBrown', 'lowerBodyGreen', 'lowerBodyGrey', 'lowerBodyOrange', 'lowerBodyPink', 'lowerBodyPurple', 'lowerBodyRed', 'lowerBodyWhite', 'lowerBodyYellow'],
                      key=lambda x: result[x],
                      reverse=True
                    )
upper_color = upper_colors[0][9:].lower()
down_color = down_colors[0][9:].lower()

# go back to master
navigation.move_to(*poses.wall_l)
navigation.move_to(*poses.master)

# report
out_txt = f"the second mate i found is named {person2_name}, located besides the fridge, wearing {upper_color} upper clothes, and wearing {down_color} down clothes"
respeaker.say(out_txt)
print(f"spoken text: {out_txt}")


# wait for command to go to living room
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "findmates":
            break
    rate.sleep()

print("event: find my mates")
respeaker.say("Okay, I will find your mates")

# navigate to television
navigation.move_to(*poses.wall_r)

# detect person
target = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    dets = yolo.forward(frame)[0]['det']
    dets = filter_detections(dets, "person")
    dets = filter_dis(dets, depth, 4500)

    if dets:
        dets = sorted(dets, key=lambda det: det[0], reverse=True)
        target = dets[0]
        x1, y1, x2, y2, conf, cls = map(int, target)
        person_img = frame[y1:y2, x1:x2]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    rate.sleep()

print("event: detected person")

# approach the person
cx, cy = int(target[0] + target[2]) // 2, int(target[1] + target[3]) // 2
dis = up_cam.get_euclidean_distance(cx, cy, depth) / 1000
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)

chassis.turn(angle, np.radians(20))
chassis.move_for(dis - 1, 0.2)

print("event: approached person")

# ask for the name
respeaker.say("what is your name")

person3_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["mynameis", "ENTITYname"] and "name" not in session.missing_slots:
            person3_name = session.get_slot_value("name")
            break
    rate.sleep()

print(f"event: received name: {person3_name}")
respeaker.say(f"nice to meet you, {person3_name}")

# process image
result = PETA.forward(person_img)[0]
result = {k: v for k, v in zip(PETA.classes, result)}
hair_lengths = sorted(["hairShort", "hairLong", "hairBald"],
                      key=lambda x: result[x],
                      reverse=True
                     )
hair_length = hair_lengths[0][4:].lower()

# go back to master
navigation.move_to(*poses.wall_r)
navigation.move_to(*poses.master)

# report
out_txt = f"the third mate i found is named {person3_name}, located besides the television, and its hair is {hair_length}"
respeaker.say(out_txt)
print(f"spoken text: {out_txt}")

