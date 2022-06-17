#!/usr/bin/env python3
import cv2

camera = cv2.VideoCapture(0)
while camera.isOpened():
    success, frame = camera.read()
    if not success:
        break

    h, w, c = frame.shape
    print("Width: %d, Height: %d, Channels: %d" % (w, h, c))

    frame[:h//2, :w//2, (1, 2)] = 0
    frame[:h//2, w//2:, (0, 2)] = 0
    frame[h//2:, :w//2, (0, 1)] = 0

    cv2.imshow("frame", frame)
    key_code = cv2.waitKey(1)
    if key_code in [27, ord('q')]:
        break
camera.release()
cv2.destroyAllWindows()

