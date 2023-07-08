#!/usr/bin/env python3
import rospy
import core
import math
import numpy as np
import cv2
from PIL import Image
import time
from easydict import EasyDict as edict


rospy.init_node("receptionist")
print("node started")

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
chassis = core.Chassis()
respeaker = core.Respeaker(verbose=True)
assistant = core.nlu.Assistant()
assistant.set_dataset(core.nlu.Dataset().from_yaml("rec_dataset.yaml"))
session = assistant.session
yolo = core.openvino_models.Yolov8("yolov8n")
navigation = core.Navigation()
PA100k = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR")
PETA = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR_PETA")
rate = rospy.Rate(30)

poses = edict()
poses.init = (3.684046745300293, -0.9851312637329102, 0.3399)
poses.fbi = (4.470449924468994, -0.5165722966194153, 0.01185)
poses.left = (3.271702766418457, 1.0125932693481445, 3.024)
poses.right = (3.271702766418457, 1.0125932693481445, -0.03815)
poses.mid = (3.2241295694938557, 0.7275529434384217, 1.605)


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


respeaker.say("I am ready")

guest1_img = None
# guest1 trigger
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    detections = yolo.forward(frame)[0]['det']
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 1500)

    if detections:
        target = detections[0]
        x1, y1, x2, y2, conf, cls = map(int, target)
        guest1_img = frame[y1:y2, x1:x2]
        yolo.draw_bounding_box({'det': detections}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    rate.sleep()

# approach the guest
navigation.move_to(*poses.fbi)

# ask for the name
respeaker.say("welcome to the party, what is your name")

guest1_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["tellName", "ENTITYname"] and "name" not in session.missing_slots:
            guest1_name = session.get_slot_value("name")
            break
    
    rate.sleep()

# ask for the favourite drink
respeaker.say(f"okay {guest1_name}, what is your favourite drink")

guest1_drink = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["tellDrink", "ENTITYdrink"] and "drink" not in session.missing_slots:
            guest1_drink = session.get_slot_value("drink")
            break
    
    rate.sleep()

# go to the middle of the living room
respeaker.say("Okay, please follow me")
navigation.move_to(*poses.mid)

# # turn right slowly and scan person
# flag = False
# while not rospy.is_shutdown():
#     chassis.move(0, np.radians(-10))
#     frame = up_cam.read_rgb()
#     depth = up_cam.read_depth()

#     detections = yolo.forward(frame)[0]['det']
#     detections = filter_detections(detections, "person")
#     detections = filter_dis(detections, depth, 3000)
#     detections = sorted(detections, key=lambda x: x[0], reverse=True)

#     for det in detections:
#         cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
#         if abs(cx - 320) < 200:
#             yolo.draw_bounding_box({'det': [det]}, frame)
#             evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#             evi.show()
#             flag = True
#             break
    
#     if flag:
#         break
    
#     yolo.draw_bounding_box({'det': detections}, frame)
#     cv2.imshow("frame", frame)
#     cv2.waitKey(1)

#     rate.sleep()

# chassis.move(0, 0)
# time.sleep(1)

# introduce the first guest
respeaker.say("hello master, new guest here")
time.sleep(3)
chassis.turn(3.14159, np.radians(10))
respeaker.say(f"this is {guest1_name}, who prefers {guest1_drink}")
time.sleep(5)

# slowly turn left to find empty seat
target = None
while not rospy.is_shutdown():
    chassis.move(0, np.radians(20))

    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    dets = yolo.forward(frame)[0]['det']

    chairs = filter_detections(dets, "chair")
    persons = filter_detections(dets, "person")
    persons = filter_dis(persons, depth, 3000)

    seat = []
    for chair in chairs:
        cx = (chair[0] + chair[2]) / 2
        flag = 150 < cx < 640 - 150                                                                                     
        for person in persons:
            if max(chair[0], person[0]) < min(chair[2], person[2]):
                flag = False
                break
        if flag:
            seat.append(chair)
    
    if seat:
        target = seat[0]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    yolo.draw_bounding_box({'det': chairs}, frame)
    yolo.draw_bounding_box({'det': persons}, frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    rate.sleep()

chassis.move(0, 0)
time.sleep(0.5)

cx, cy = (target[0] + target[2]) / 2, (target[1] + target[3]) / 2
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)
angle -= np.radians(15)
chassis.turn(angle, np.radians(20))

respeaker.say("Please take a seat in this empty chair in front of me")

time.sleep(3)

# navigate to standby pose
navigation.move_to(*poses.init)

# guest2 trigger
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    detections = yolo.forward(frame)[0]['det']
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 2000)

    if detections:
        yolo.draw_bounding_box({'det': detections}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    rate.sleep()

# approach the guest
navigation.move_to(*poses.fbi)

# ask for the name
respeaker.say("welcome to the party, what is your name")

guest2_name = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["tellName", "ENTITYname"] and "name" not in session.missing_slots:
            guest2_name = session.get_slot_value("name")
            break
    
    rate.sleep()

# ask for the favourite drink
respeaker.say(f"okay {guest2_name}, what is your favourite drink")

guest2_drink = None
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        print(session.intent_name)
        if session.intent_name in ["tellDrink", "ENTITYdrink"] and "drink" not in session.missing_slots:
            guest2_drink = session.get_slot_value("drink")
            if guest2_drink != "drink":
                break
    
    rate.sleep()

# go to the middle of the living room
respeaker.say("Okay, please follow me")
navigation.move_to(*poses.mid)

# slowly turn right and scan the people
# flag = False
# while not rospy.is_shutdown():
#     chassis.move(0, np.radians(-20))
#     frame = up_cam.read_rgb()
#     depth = up_cam.read_depth()

#     detections = yolo.forward(frame)[0]['det']
#     detections = filter_detections(detections, "person")
#     detections = filter_dis(detections, depth, 3000)

#     for det in detections:
#         cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
#         if abs(cx - 320) < 40:
#             yolo.draw_bounding_box({'det': [det]}, frame)
#             evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#             evi.show()
#             flag = True
#             break
    
#     if flag:
#         break
    
#     yolo.draw_bounding_box({'det': detections}, frame)
#     cv2.imshow("frame", frame)
#     cv2.waitKey(1)

#     rate.sleep()

# chassis.move(0, 0)
# time.sleep(1)

# respeaker.say("hello")

# chassis.turn_to(poses.right[2], np.radians(20))

# flag = False
# while not rospy.is_shutdown():
#     chassis.move(0, np.radians(10))
#     frame = up_cam.read_rgb()
#     depth = up_cam.read_depth()

#     detections = yolo.forward(frame)[0]['det']
#     detections = filter_detections(detections, "person")
#     detections = filter_dis(detections, depth, 3000)

#     for det in detections:
#         cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
#         if abs(cx - 320) < 200:
#             yolo.draw_bounding_box({'det': [det]}, frame)
#             evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#             evi.show()
#             flag = True
#             break
    
#     if flag:
#         break
    
#     yolo.draw_bounding_box({'det': detections}, frame)
#     cv2.imshow("frame", frame)
#     cv2.waitKey(1)

#     rate.sleep()

# chassis.move(0, 0)
# time.sleep(1)

respeaker.say("hello guys")
time.sleep(3)
chassis.turn(3.14159, np.radians(10))
respeaker.say(f"this is {guest2_name}, who prefers {guest2_drink}")
time.sleep(5)

# slowly turn left to find empty seat
target = None
while not rospy.is_shutdown():
    chassis.move(0, np.radians(20))

    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()
    
    dets = yolo.forward(frame)[0]['det']

    chairs = filter_detections(dets, "chair")
    persons = filter_detections(dets, "person")
    persons = filter_dis(persons, depth, 3000)

    seat = []
    for chair in chairs:
        cx = (person[0] + person[2]) / 2
        flag = 150 < cx < 640 - 150
        for person in persons:
            if max(chair[0], person[0]) < min(chair[2], person[2]):
                flag = False
                break
        if flag:
            seat.append(chair)
    
    if seat:
        target = seat[0]
        yolo.draw_bounding_box({'det': [target]}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break
    
    yolo.draw_bounding_box({'det': chairs}, frame)
    yolo.draw_bounding_box({'det': persons}, frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    rate.sleep()

chassis.move(0, 0)
time.sleep(0.5)

cx, cy = (target[0] + target[2]) / 2, (target[1] + target[3]) / 2
angle = -np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)
angle -= np.radians(15)
chassis.turn(angle, np.radians(20))

respeaker.say("Please take a seat in this empty chair in front of me")

time.sleep(5)


PA100k_result = PA100k.forward(guest1_img)[0]
PETA_result = PETA.forward(guest1_img)[0]
PA100k_result = {k: v for k, v in zip(PA100k.classes, PA100k_result)}
PETA_result = {k: v for k, v in zip(PETA.classes, PETA_result)}
gender = "male" if PA100k_result['Female'] < 0.4 else "female"
upper_colors = sorted(['upperBodyBlack', 'upperBodyBlue', 'upperBodyBrown', 'upperBodyGreen', 'upperBodyGrey', 'upperBodyOrange', 'upperBodyPink', 'upperBodyPurple', 'upperBodyRed', 'upperBodyWhite', 'upperBodyYellow'],
                      key=lambda x: PETA_result[x],
                      reverse=True
                     )
down_colors = sorted(['lowerBodyBlack', 'lowerBodyBlue', 'lowerBodyBrown', 'lowerBodyGreen', 'lowerBodyGrey', 'lowerBodyOrange', 'lowerBodyPink', 'lowerBodyPurple', 'lowerBodyRed', 'lowerBodyWhite', 'lowerBodyYellow'],
                      key=lambda x: PETA_result[x],
                      reverse=True
                    )
hair_colors = sorted(['hairBlack', 'hairBlue', 'hairBrown', 'hairGreen', 'hairGrey', 'hairOrange', 'hairPink', 'hairPurple', 'hairRed', 'hairWhite', 'hairYellow'],
                     key=lambda x: PETA_result[x],
                     reverse=True
                    )
upper_color = upper_colors[0][9:].lower()
down_color = down_colors[0][9:].lower()
hair_color = hair_colors[0][4:].lower()
age_dets = sorted(['AgeOver60', 'Age18-60', 'AgeLess18'],
                  key=lambda x: PA100k_result[x],
                  reverse=True
                 )
if age_dets[0] == "AgeOver60": age = "old"
if age_dets[0] == "Age18-60": age = "adult"
if age_dets[0] == "AgeLess18": age = "young"

respeaker.say(f"guest 1 is a {age} {gender}, wearing {upper_color} upper clothes, and {down_color} bottom clothes, who has a {hair_color} colored hair")
