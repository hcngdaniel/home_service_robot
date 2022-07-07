#!/usr/bin/env python3
import rospy
import cv2
import core
from hccd.api import detect_code
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import pyttsx3
import yaml


rospy.init_node("live_demo")
astra = core.Astra()
assistant = core.nlu.Assistant().load('/home/hcng/catkin_ws/src/home_service_robot/scripts/live_demo/assistant.tar')

person_count = 0
pc_frame = np.zeros((480, 640, 3), dtype="uint8")


def person_count_cb(msg):
    global person_count
    person_count = int(msg.data)


def pc_frame_cb(msg):
    global pc_frame
    pc_frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")


person_count_sub = rospy.Subscriber("/personcnt", Int32, callback=person_count_cb)
pc_imgstream_sub = rospy.Subscriber("/frame_for_person_count", Image, callback=pc_frame_cb)

start_time = time.time() + 99999999
flag = True
while not rospy.is_shutdown():
    cv2.imshow("frame", pc_frame)
    key = cv2.waitKey(16)
    if key in [27, ord('q')]:
        break
    if person_count == 1 and flag:
        start_time = time.time()
        flag = False
    if time.time() - start_time > 0.5:
        break
pyttsx3.speak("hello, please show me your health code")

green_chain = 0
yellow_chain = 0
red_chain = 0
while not rospy.is_shutdown():
    frame = astra.read_rgb()
    code_frame, color = detect_code(frame)
    cv2.imshow("frame", code_frame)
    key = cv2.waitKey(16)
    if key in [27, ord('q')]:
        break
    if color is not None:
        if color == "green":
            green_chain += 1
            yellow_chain /= 2
            red_chain /= 2
        if color == "yellow":
            yellow_chain += 1
            red_chain /= 2
            green_chain /= 2
        if color == "red":
            red_chain += 1
            yellow_chain /= 2
            green_chain /= 2
    if green_chain >= 20:
        break

pyttsx3.speak("your health code is green, you may pass")

while not rospy.is_shutdown():
    text = input()
    res = assistant.session.request(text)
    if res.intent_name == "recommendBook":
        break
    rospy.Rate(20).sleep()

pyttsx3.speak("Okay, what is your name")

eps = 1e-10
user = input()

user_file = f'Users/{user}.yaml'
with open(user_file, 'r') as file:
    user_yaml = dict(yaml.safe_load(file))
user_read = user_yaml['read']
del user_yaml['read']
user_read = map(lambda x: x.lower(), user_read)
user_sum = sum(user_yaml.values())
user_poss = {k: v / user_sum + eps for k, v in user_yaml.items()}

public_file = 'Users/Public.yaml'
with open(public_file, 'r') as file:
    public_yaml = dict(yaml.safe_load(file))
public_sum = sum(public_yaml.values())
public_poss = {k: v / public_sum + eps for k, v in public_yaml.items()}

for k in user_poss.keys():
    user_poss[k] = (user_poss[k] * public_poss[k]) / \
                   (user_poss[k] * public_poss[k] + (1 - user_poss[k]) * (1 - public_poss[k]))

categories = sorted(user_poss.items(), key=lambda x: x[1], reverse=True)

book_list = []

with open(f'Categories/{categories[0][0]}.txt', 'r') as f:
    i = 0
    while i < 3:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

with open(f'Categories/{categories[1][0]}.txt', 'r') as f:
    i = 0
    while i < 2:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

with open(f'Categories/{categories[2][0]}.txt', 'r') as f:
    i = 0
    while i < 1:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

print(book_list)
book_list = list(map(lambda x: x.lower(), book_list))

pyttsx3.speak(f"Alright, {user}, here are your recommended books")

book1 = cv2.imread(f"pictures/{book_list[0]}.jpeg")
book1 = cv2.resize(book1, (190, 281))

book2 = cv2.imread(f"pictures/{book_list[1]}.jpeg")
book2 = cv2.resize(book2, (190, 281))

book3 = cv2.imread(f"pictures/{book_list[2]}.jpeg")
book3 = cv2.resize(book3, (190, 281))

book4 = cv2.imread(f"pictures/{book_list[3]}.jpeg")
book4 = cv2.resize(book4, (190, 281))

book5 = cv2.imread(f"pictures/{book_list[4]}.jpeg")
book5 = cv2.resize(book5, (190, 281))

book6 = cv2.imread(f"pictures/{book_list[5]}.jpeg")
book6 = cv2.resize(book6, (190, 281))

canva = np.zeros((281 * 2, 190 * 3, 3), dtype="uint8")
canva[0:281, 0:190, :] = book1
canva[0:281, 190:190*2, :] = book2
canva[0:281, 190*2:190*3, :] = book3
canva[281:281*2, 0:190, :] = book4
canva[281:281*2, 190:190*2, :] = book5
canva[281:281*2, 190*2:190*3, :] = book6
cv2.imshow("frame", canva)
cv2.waitKey(-1)

book_want = "None"
while not rospy.is_shutdown():
    text = input()
    if text:
        res = assistant.session.request(text)
        if res.intent_name == "bringMeToBook":
            if 'book' in res.missing_slots:
                pyttsx3.speak("I don't know which book you meant")
            else:
                book_want = res.parse_result['slots'][0]['value']['value']
                pyttsx3.speak(f"Okay, I will bring you to {book_want}")
                break
    cv2.imshow("frame", canva)
    cv2.waitKey(16)

print(book_want)
