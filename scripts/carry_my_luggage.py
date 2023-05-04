#!/usr/bin/env python3
import rospy
import time
import core
import cv2
import numpy as np
from cv_bridge import CvBridge


MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = np.radians(20)

rospy.init_node("carry_my_luggage")

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
chassis = core.Chassis()
respeaker = core.Respeaker()
assistant = core.nlu.Assistant.load("cml_assistant.ass")
session = assistant.session
yolo = core.openvino_models.Yolov8("yolov8n")
hb_model = core.openvino_models.Yolov8("handbag")
hb_model.classes = {0: "handbag"}
navigation = core.Navigation()
manipulator = core.Manipulator()

time.sleep(3)

lpid = core.utils.PID(-0.001, 0, 0)
apid = core.utils.PID(0.5, 0, 0)
home_pose = (0.000, -1.049, 0.357, 0.703)
init_pose = (0.000, 0.012, 0.035, 0.009)
down_pose = (0.000, 0.784, 0.909, -0.197)
ready_pose = (0.000, 0.784, 0.909, -1.447)
grip_pose = (0.000, 1.284, -0.041, -1.447)
take_pose = (0.000, 0.734, -0.041, -1.747)
drop_pose = (0.000, 0.012, 0.035, 1.500)
start_point = navigation.current_pose.pose.pose

print("node started!")

manipulator.set_joint(*init_pose, 1.5)
manipulator.set_joint(*home_pose, 1.5)
manipulator.close_gripper(1.5)

respeaker.say("I am ready")

direction_str = ""
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        print(text)
        session.request(text)
        print(session.intent_name)
        if session.intent_name == "takeluggage":
            if "direction" not in session.missing_slots:
                print('ok')
                direction_str = session.get_slot_value("direction")
                break
    rospy.Rate(16).sleep()

respeaker.say(f"okay, I will take the luggage on the {direction_str}")
print("event: take my luggage")

while not rospy.is_shutdown():
    frame = down_cam.read_rgb()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    detections = hb_model.forward(frame)[0]
    bags = []
    for detection in detections['det']:
        if detection[4] > 0.75:
            bags.append(detection)
    targets = []
    for bag in bags:
        if direction_str == 'right' and bag[0] + bag[2] >= 640:
            targets.append(bag)
        if direction_str == 'left' and bag[0] + bag[2] < 640:
            targets.append(bag)
    hb_model.draw_bounding_box({'det': targets}, frame)

    if targets:
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        target = targets[0]
        depth = down_cam.read_depth()
        depth = cv2.rotate(depth, cv2.ROTATE_180)
        tx, ty, tz = down_cam.get_real_xyz(int((target[0] + target[2]) / 2), int((target[1] + target[3]) / 2), depth)
        chassis.turn(np.radians(-90) * (tx / abs(tx)), MAX_ANGULAR_SPEED)
        time.sleep(0.2)
        chassis.move_for(abs(tx) / 1000, MAX_LINEAR_SPEED * 0.5)
        time.sleep(0.2)
        chassis.turn(np.radians(90) * (tx / abs(tx)), MAX_ANGULAR_SPEED)
        time.sleep(0.2)
        chassis.move_for(tz / 1000 - 0.25, MAX_LINEAR_SPEED * 0.5)
        time.sleep(0.2)
        break

    cv2.imshow("frame", frame)
    rospy.Rate(16).sleep()
    cv2.waitKey(1)

manipulator.set_joint(*init_pose, 1.5)
manipulator.set_joint(*down_pose, 1.5)
manipulator.set_joint(*ready_pose, 1.5)
manipulator.open_gripper(1.5)
manipulator.set_joint(*grip_pose, 1.5)
chassis.move_for(0.05, MAX_LINEAR_SPEED * 0.4)
manipulator.set_joint(*take_pose, 1.5)
manipulator.close_gripper(1.5)
time.sleep(1)


print("event: finished gripping")

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        if session.intent_name == "followme":
            break
    rospy.Rate(16).sleep()

print("event: follow me")
respeaker.say("okay, I will follow you")

while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        if session.intent_name == "arrive":
            break

    frame = up_cam.read_rgb()

    detections = yolo.forward(frame)[0]
    persons = []
    for i, detection in enumerate(detections["det"]):
        cls = yolo.classes[int(detection[5])]
        if cls == "person":
            persons.append(detection)
    yolo.draw_bounding_box(detections, frame)

    depth_img = up_cam.read_depth()
    cv2.imshow("depth", depth_img)

    if persons:
        target = [float('inf'), float('inf'), float('inf')]
        for person in persons:
            px = int((person[0] + person[2]) / 2)
            py = int((person[1] + person[3]) / 2)
            coord = up_cam.get_real_xyz(px, py, depth_img)
            if coord[2] < target[2]:
                target = coord

        lspeed = lpid(1000, target[2])
        lspeed = min(lspeed, MAX_LINEAR_SPEED)
        lspeed = max(lspeed, -MAX_LINEAR_SPEED)

        aspeed = 0
        if target[2] != 0:
            adiff = np.arctan(target[0] / (target[2]))
            aspeed = apid(0, adiff)
            aspeed = min(aspeed, MAX_ANGULAR_SPEED)
            aspeed = max(aspeed, -MAX_ANGULAR_SPEED)

        chassis.move(lspeed, aspeed)

    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)

    if key in [ord('q'), 27]:
        break
    rospy.Rate(30).sleep()

chassis.move(0, 0)
respeaker.say("okay, I will put down the luggage")
print("event: put down luggage")

manipulator.set_joint(*init_pose, 1.5)
manipulator.open_gripper(1.5)
manipulator.set_joint(*drop_pose, 0.5)
time.sleep(3)
manipulator.set_joint(*init_pose, 0.5)
manipulator.set_joint(*home_pose, 1.5)


while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text:
        session.request(text)
        if session.intent_name == "goback":
            break
    rospy.Rate(16).sleep()

respeaker.say("okay, I will go back to my stating position")

time.sleep(1)

navigation.move_to_pose(start_point)

cv2.destroyAllWindows()
