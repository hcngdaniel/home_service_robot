#!/usr/bin/env python3
import cv2
import core
import rospy


yolo = core.openvino_models.Yolov8("yolov8n")
resnet50_PAR = core.openvino_models.resnet50_PAR()

cap = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    success, frame = cap.read()
    dets = yolo.forward(frame)[0]['det']
    persons = []
    for det in dets:
        if det[5] == 0:
            persons.append(det)
    if persons:
        person = persons[0]
        x1, y1, x2, y2, conf, cls = map(int, person)
        img = frame[y1:y2, x1:x2]
        cv2.imshow("img", img)
        result = resnet50_PAR.forward(img)[0]
        result = [(k, v) for k, v in zip(resnet50_PAR.classes, result)]
        result = sorted(result, key=lambda x: x[1], reverse=True)
        print(result)
    cv2.waitKey(30)
