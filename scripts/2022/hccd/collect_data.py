#!/usr/bin/env python3
import cv2
import pyzbar.pyzbar


cap = cv2.VideoCapture(0)
g_idx = 112
y_idx = 89
r_idx = 190
while cap.isOpened():
    success, frame = cap.read()
    key = cv2.waitKey(16)
    result = pyzbar.pyzbar.decode(frame)
    try:
        result = result[0]
    except:
        continue
    left, top, width, height = result.rect
    img = frame[top:top + height, left:left + width, :].copy()
    if key == ord('g'):
        cv2.imwrite(f"data/green/{g_idx}.jpg", cv2.resize(img, (640, 480)))
        g_idx += 1
    if key == ord('y'):
        cv2.imwrite(f"data/yellow/{y_idx}.jpg", cv2.resize(img, (640, 480)))
        y_idx += 1
    if key == ord('r'):
        cv2.imwrite(f"data/red/{r_idx}.jpg", cv2.resize(img, (640, 480)))
        r_idx += 1
    cv2.imshow("frame", frame)
