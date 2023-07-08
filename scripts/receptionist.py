#!/usr/bin/env python3
import rospy
import core
import math
import numpy as np
import cv2
from PIL import Image
import time


rospy.init_node("receptionist")
print("node started")

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
chassis = core.Chassis()
respeaker = core.Respeaker()
assistant = core.nlu.Assistant()
assistant.set_dataset(core.nlu.Dataset().from_yaml("rec_dataset.yaml"))
session = assistant.session
yolo = core.openvino_models.Yolov8("yolov8n")
navigation = core.Navigation()
manipulator = core.Manipulator()


main_hall = (2.62, -5.0, 3.14159265358979323846)
master_pos = (0.0, 0.0, 0.0)
init_pos = (0.0, 0., 0.)

home_pose = (0.000, -1.049, 0.357, 0.703)
init_pose = (0.000, 0.012, 0.035, 0.009)

manipulator.set_joint(*init_pose, 1.5)
manipulator.set_joint(*home_pose, 1.5)

respeaker.say("I am ready")

while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()

    detections = yolo.forward(frame)[0]
    persons = []
    for i, detection in enumerate(detections["det"]):
        cls = yolo.classes[int(detection[5])]
        if cls == "person":
            persons.append(detection)
    yolo.draw_bounding_box({'det': persons}, frame)
    target = []
    for person in persons:
        if person[4] < 0.75: continue
        cx, cy = int(person[0] + person[2]) // 2, int(person[1] + person[3]) // 2
        dis = up_cam.get_euclidean_distance(cx, cy, depth)
        if dis < 2000:
            target.append(person)
    if target:
        evi = Image.fromarray(cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        break
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(30).sleep()

respeaker.say("Welcome to the party! What is your name?")
while not rospy.is_shutdown():
    voice = respeaker.get_voice()
    text = voice.text
    direction = voice.direction
    if text:
        chassis.turn(direction * math.pi / 180, np.radians(20))
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "tellName":
            if "name" not in session.missing_slots:
                name = session.get_slot_value("name")
                break

    rospy.Rate(16).sleep()

print("Event: received name")

respeaker.say("What is your favourite drink?")

while not rospy.is_shutdown():
    voice = respeaker.get_voice()
    text = voice.text
    direction = voice.direction
    if text:
        chassis.turn(np.radians(direction), np.radians(40))
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "tellDrink":
            if "drink" not in session.missing_slots:
                drink = session.get_slot_value("drink")
                break

    rospy.Rate(16).sleep()

respeaker.say(f"Okay {name}, Please follow me to the main hall")

print(f"{name}'s favourite drink is {drink}")

navigation.move_to(*main_hall)

target = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()

    dets = yolo.forward(frame)[0]

    chairs = []
    persons = []
    for det in dets["det"]:
        cls = yolo.classes[int(det[5])]
        if cls == "chair" and det[4] > 0.4:
            chairs.append(det)
        if cls == "person" and det[4] > 0.6:
            persons.append(det)

    tframe = frame.copy()
    yolo.draw_bounding_box({'det': persons + chairs}, frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    targets = []
    for chair in chairs:
        flag = True
        for person in persons:
            if max(chair[0], person[0]) < min(chair[2], person[2]):
                flag = False
                break
        if flag:
            targets.append(chair)

    if targets:
        yolo.draw_bounding_box({'det': [targets[0]]}, tframe)
        evi = Image.fromarray(cv2.cvtColor(tframe.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        evi = Image.fromarray(cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        target = targets[0]
        break

    rospy.Rate(30).sleep()

cx, cy = (target[0] + target[2]) / 2, (target[1] + target[3]) / 2
angle = np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)
chassis.turn(-angle, np.radians(20))

respeaker.say("Please take a seat in this empty chair in front of me")

manipulator.set_joint(*init_pose, 1.5)

time.sleep(3)

# navigation.move_to(*master_pos)

respeaker.say(f"this is {name}, and their favourite drink is {drink}")

navigation.move_to(*init_pos)

while not rospy.is_shutdown():
    frame = up_cam.read_rgb()
    depth = up_cam.read_depth()

    detections = yolo.forward(frame)[0]
    persons = []
    for i, detection in enumerate(detections["det"]):
        cls = yolo.classes[int(detection[5])]
        if cls == "person":
            persons.append(detection)
    yolo.draw_bounding_box({'det': persons}, frame)
    target = []
    for person in persons:
        if person[4] < 0.75: continue
        cx, cy = int(person[0] + person[2]) // 2, int(person[1] + person[3]) // 2
        dis = up_cam.get_euclidean_distance(cx, cy, depth)
        if dis < 2000:
            target.append(person)
    if target:
        evi = Image.fromarray(cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        break
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(30).sleep()

respeaker.say("Welcome to the party! What is your name?")
while not rospy.is_shutdown():
    voice = respeaker.get_voice()
    text = voice.text
    direction = voice.direction
    if text:
        chassis.turn(direction * math.pi / 180, np.radians(20))
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "tellName":
            if "name" not in session.missing_slots:
                name = session.get_slot_value("name")
                break

    rospy.Rate(16).sleep()

print("Event: received name")

respeaker.say("What is your favourite drink?")

while not rospy.is_shutdown():
    voice = respeaker.get_voice()
    text = voice.text
    direction = voice.direction
    if text:
        chassis.turn(np.radians(direction), np.radians(40))
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "tellDrink":
            if "drink" not in session.missing_slots:
                drink = session.get_slot_value("drink")
                break

    rospy.Rate(16).sleep()

respeaker.say(f"Okay {name}, Please follow me to the main hall")

print(f"{name}'s favourite drink is {drink}")

navigation.move_to(*main_hall)

target = None
while not rospy.is_shutdown():
    frame = up_cam.read_rgb()

    dets = yolo.forward(frame)[0]

    chairs = []
    persons = []
    for det in dets["det"]:
        cls = yolo.classes[int(det[5])]
        if cls == "chair" and det[4] > 0.4:
            chairs.append(det)
        if cls == "person" and det[4] > 0.6:
            persons.append(det)

    tframe = frame.copy()
    yolo.draw_bounding_box({'det': persons + chairs}, frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    targets = []
    for chair in chairs:
        flag = True
        for person in persons:
            if max(chair[0], person[0]) < min(chair[2], person[2]):
                flag = False
                break
        if flag:
            targets.append(chair)

    if targets:
        yolo.draw_bounding_box({'det': [targets[0]]}, tframe)
        evi = Image.fromarray(cv2.cvtColor(tframe.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        evi = Image.fromarray(cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB))
        evi.show()
        target = targets[0]
        break

    rospy.Rate(30).sleep()

cx, cy = (target[0] + target[2]) / 2, (target[1] + target[3]) / 2
angle = np.arctan((cx - 320) * np.tan(np.radians(30)) / 320)
chassis.turn(-angle, np.radians(20))

respeaker.say("Please take a seat in this empty chair in front of me")

manipulator.set_joint(*init_pose, 1.5)

time.sleep(3)

# navigation.move_to(*master_pos)

respeaker.say(f"this is {name}, and their favourite drink is {drink}")
