#!/usr/bin/env python3
import rospy
import core
import numpy as np
from PIL import Image
import cv2
import time


rospy.init_node("point_bag")

down_cam = core.Astra()
respeaker = core.Respeaker()
openpose = core.openvino_models.HumanPoseEstimation()
hb_model = core.openvino_models.Yolov8("handbag3")
hb_model.classes = {0: "handbag"}

def filter_conf_openpose(detections, node_id, threshold):
    res = []
    for detection in detections:
        if detection[node_id][2] >= threshold:
            res.append(detection)
    return res

def filter_dis_openpose(detections, node_id, depth_img, threshold):
    res = []
    for detection in detections:
        dis = down_cam.get_euclidean_distance(
            int(detection[node_id][0]),
            int(detection[node_id][1]),
            depth_img,
        )
        if dis <= threshold:
            res.append(detection)
    return res

# while not rospy.is_shutdown():
#     text = respeaker.get_text()
#     if text:
#         print(text)
#         if "luggage" in text.lower():
#             break

print("event: pick up bag triggered")

start_time = time.time()
run_time = 3
bags = []
bags_det = []
while not rospy.is_shutdown():

    frame = down_cam.read_rgb()
    depth = down_cam.read_depth()

    detections = hb_model.forward(frame)
    hb_model.draw_bounding_box(detections[0], frame)
    detections = detections[0]['det']

    for det in detections:
        if det[4] < 0.25:
            continue
        cx, cy = int(det[0] + det[2]) // 2, int(det[1] + det[3]) // 2
        x, y, z = down_cam.get_real_xyz(cx, cy, depth)
        bags_det.append(det)
        bags.append((x, y, z))

    if time.time() - start_time >= run_time:
        hb_model.draw_bounding_box({'det': bags_det}, frame)
        evi = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        evi.show()
        break

elbow, wrist = None, None
target_bag = None
while not rospy.is_shutdown():
    frame = down_cam.read_rgb()
    depth = down_cam.read_depth()
    
    detections = openpose.forward(frame)
    openpose.draw_poses(frame, detections, 0.5)
    cv2.imshow("frmae", frame)
    cv2.waitKey(1)

    detections = filter_conf_openpose(detections, 8, 0.5)
    detections = filter_conf_openpose(detections, 10, 0.5)
    detections = filter_dis_openpose(detections, 8, depth, 2500)
    detections = filter_dis_openpose(detections, 10, depth, 2500)

    for target in detections:
        elbow = target[8]
        wrist = target[10]
        elbow = down_cam.get_real_xyz(elbow[0], elbow[1], depth)
        wrist = down_cam.get_real_xyz(wrist[0], wrist[1], depth)
        direction = (wrist[0] - elbow[0], wrist[1] - elbow[1], wrist[2] - elbow[2])
        targets = []
        for bag in bags:
            error = (bag[0] - wrist[0], bag[1] - wrist[1], bag[2] - wrist[2])
            prod = direction[0] * error[0] + direction[1] * error[1] + direction[2] * error[2]
            sim = prod / (core.utils.distance((0, 0, 0), direction) * core.utils.distance((0, 0, 0), error))
            targets.append((sim, bag))
        targets = sorted(targets, key=lambda x: x[0])
        if targets[0][0] < np.radians(5):
            target_bag = targets[1]
            break
    
    if target_bag is not None:
        break

    rospy.Rate(30).sleep()

