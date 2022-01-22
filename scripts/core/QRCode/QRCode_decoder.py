#!/usr/bin/env python3
from pyzbar.pyzbar import decode
import numpy as np
import typing
import PIL.Image
import cv2


class QRCodeDecoder:
    @staticmethod
    def decode(img: typing.Union[np.ndarray, str]) -> typing.List:
        if isinstance(img, np.ndarray) or isinstance(img, PIL.Image.Image):
            return decode(img)
        if isinstance(img, str):
            return decode(cv2.imread(img))


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        success, frame = cap.read()
        codes = QRCodeDecoder.decode(frame)
        for code in codes:
            pts = np.array([[point.x, point.y] for point in code.polygon])
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 0, 255), thickness=1)
            print(code.data.decode())
        cv2.imshow("frame", frame)
        if cv2.waitKey(16) in [27, ord('q')]:
            break
