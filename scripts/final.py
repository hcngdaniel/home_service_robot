#!/usr/bin/env python3
import os
os.chdir(os.path.dirname(__file__))
import rospy
import core
import time
import cv2
import numpy as np
from std_msgs.msg import Bool


PERSON_COUNTING_MAX_DEPTH = 1200
FIND_OCCUPIED_SEAT_MAX_DEPTH = 1500


rospy.init_node("final")

ucam = core.Astra("/ucam")
dcam = core.Astra("/dcam")
navigation = core.Navigation(testing=False)
# manipulator = core.Manipulator()
yolo = core.openvino_models.Yolov8("yolov8n")
respeaker = core.Respeaker(verbose=True)
chassis = core.Chassis()
assistant = core.nlu.Assistant.load("final.ass")
session = assistant.session

button_state = False
def button_cb(msg):
    global button_state
    button_state = msg.data

rospy.Subscriber("/button_state", Bool, callback=button_cb)

door_location = (-2.118, -1.060, -1.542)
seat_location = (0.9935213204058614, -0.062177164735149844, 3.14)
out_location = (-0.9127339060608182, 0.40855454559576443, -2.299852961291163)

time.sleep(3)
respeaker.say("I am ready")

def id_filter(dets, id = 0):
    ret = []
    for det in dets['det']:
        if det[5] == id:
            ret.append(det)
    return {'det': ret}

def depth_filter(dets, depth, max_depth):
    ret = []
    for det in dets['det']:
        cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
        _, _, rz = ucam.get_real_xyz(cx, cy, depth, autofill=True)
        if 0 < rz <= max_depth:
            ret.append(det)
    return {'det': ret}

def most_center(dets):
    if len(dets['det']) == 0:
        return None
    answer = dets['det'][0]
    for det in dets['det']:
        if abs(int(det[0] + det[2]) // 2 - 320) < abs(int(answer[0] + answer[2]) // 2 - 320):
            answer = det
    return answer

last_cx = float('inf')
person_count = 1
while not rospy.is_shutdown():
    frame = ucam.read_rgb()
    depth = ucam.read_depth()
    
    dets = yolo.forward(frame)[0]
    persons = id_filter(dets, id=0)
    persons = depth_filter(dets, depth, PERSON_COUNTING_MAX_DEPTH)
    yolo.draw_bounding_box(persons, frame)
    target = most_center(persons)
    if target is not None:
        cx = int(target[0] + target[2]) // 2

        if last_cx in range(320, 400) and cx in range(240, 320):
            print("event: one person passed the door")
            person_count += 1
            break

        last_cx = cx

    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(16).sleep()

print("action: bring one person to their seat")
respeaker.say("I will lead you to your seat, please follow me")

navigation.move_to(*seat_location)

occupied_seat = ""
empty_seat = ""
while not rospy.is_shutdown():
    frame = ucam.read_rgb()
    depth = ucam.read_depth()

    dets = yolo.forward(frame)[0]
    persons = id_filter(dets, id=0)
    persons = depth_filter(dets, depth, FIND_OCCUPIED_SEAT_MAX_DEPTH)
    yolo.draw_bounding_box(persons, frame)
    target = most_center(persons)
    if target is not None:
        if int(target[0] + target[2]) // 2 < 320:
            print("observation: left seat is occupied")
            occupied_seat = "left"
            empty_seat = "right"
        else:
            print("observation: right seat is occupied")
            occupied_seat = "right"
            empty_seat = "left"
        break

    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(16).sleep()

respeaker.say(f"the seat on the {occupied_seat} is occupied, so you can sit on the {empty_seat}")
time.sleep(5)
if empty_seat == "left":
    chassis.turn(np.radians(10), np.radians(20))
else:
    chassis.turn(np.radians(-10), np.radians(20))

respeaker.say("If you need any help, please call me through the button")

time.sleep(1)

navigation.move_to(*door_location)

while not rospy.is_shutdown():
    frame = ucam.read_rgb()
    depth = ucam.read_depth()
    
    dets = yolo.forward(frame)[0]
    persons = id_filter(dets, id=0)
    persons = depth_filter(dets, depth, PERSON_COUNTING_MAX_DEPTH)
    yolo.draw_bounding_box(persons, frame)

    if len(persons) != 0:
        print("observation: one more people want to come in")
        break
    
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(16).sleep()

respeaker.say("the restaurant is full, please wait outside the door")
time.sleep(1)

print("action: waiting for button")

while not rospy.is_shutdown():
    if button_state:
        break
    rospy.Rate(16).sleep()
    cv2.waitKey(1)

print("event: button pressed")

navigation.move_to(*seat_location)

respeaker.say("hello, how can I help you?")

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(f"heard: {text}")
        print(f"intent: {session.intent_name}")
        if session.intent_name == "bill":
            break
    rospy.Rate(16).sleep()

print("event: pay bill")
respeaker.say("okay, please scan the QrCode for e-payment")
qrcode = cv2.imread("demoqr.png")
cv2.imshow("code", qrcode)

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(f"heard: {text}")
        print(f"intent: {session.intent_name}")
        if session.intent_name == "leave":
            break
    rospy.Rate(16).sleep()
    cv2.waitKey(1)

print("action: bring the customer out")
respeaker.say("I will bring you out now")
navigation.move_to(*out_location)
person_count -= 1
respeaker.say("Goodbye, have a nice day")

print("action: back to door")
navigation.move_to(*door_location)
time.sleep(1)
respeaker.say("there is one empty seat in the restaurant now, you may go in")
