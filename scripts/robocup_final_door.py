#!/usr/bin/env python3
import rospy
import core
from std_msgs.msg import String
import cv2


def filter_conf(dets, threshold):
    res = []
    for det in dets:
        if det[4] > threshold:
            res.append(det)
    return res

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
        dis = camera.get_euclidean_distance(cx, cy, depth)
        if dis < threshold:
            res.append(det)
    return res

door_state = None
def state_callback(msg):
    global door_state
    print(door_state)
    door_state = msg.data

rospy.init_node("robocup_final_door")

rate = rospy.Rate(30)
camera = core.Astra("door_cam")
state_pub = rospy.Publisher("/door_states", String, queue_size=3)
state_sub = rospy.Subscriber("/robot_states", String, callback=state_callback)
yolo = core.openvino_models.Yolov8("yolov8n")

print("node started")

while not rospy.is_shutdown():
    if door_state == "person pk":
        break
    rate.sleep()

while not rospy.is_shutdown():
    frame = camera.read_rgb()
    depth = camera.read_depth()
    
    detections = yolo.forward(frame)[0]['det']
    detections = filter_conf(detections, 0.3)
    yolo.draw_bounding_box({'det': detections}, frame)
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 1500)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)

    if detections:
        break
    
    rate.sleep()

state_pub.publish("person arrived")

while not rospy.is_shutdown():
    if door_state == "first person ready to go":
        break
    rate.sleep()

while not rospy.is_shutdown():
    frame = camera.read_rgb()
    depth = camera.read_depth()
    
    detections = yolo.forward(frame)[0]['det']
    detections = filter_conf(detections, 0.3)
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 1500)

    if detections:
        break
    
    rate.sleep()

state_pub.publish("errand1 arrived")

while not rospy.is_shutdown():
    if door_state == "second person ready to go":
        break
    rate.sleep()

while not rospy.is_shutdown():
    frame = camera.read_rgb()
    depth = camera.read_depth()
    
    detections = yolo.forward(frame)[0]['det']
    detections = filter_conf(detections, 0.3)
    detections = filter_detections(detections, "person")
    detections = filter_dis(detections, depth, 1500)

    if detections:
        break
    
    rate.sleep()

state_pub.publish("errand2 arrived")
