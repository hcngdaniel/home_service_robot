#!/usr/bin/env python3
import torch

from model import Model
from dataset import labels

import numpy as np
import cv2
from pyzbar.pyzbar import decode

device = "cuda" if torch.cuda.is_available() else "cpu"
print(device)
model = Model()
model.load_state_dict(torch.load('model-3.pth'))
model.eval()
model.to(device)


def detect_code(frame):
    decoded = decode(frame)
    color = None
    for code in decoded:
        left, top, width, height = code.rect
        img = frame[top:top + height, left:left + width, :].copy()
        if not img.any():
            continue
        img = cv2.resize(img, (640, 480))
        img = img / 255
        img = np.transpose(img, (2, 0, 1))
        x = torch.as_tensor(img, dtype=torch.float32)
        x = torch.unsqueeze(x, 0)
        x = x.to(device)
        h = model(x)
        poss = h.cpu().detach().numpy()[0]
        color = labels[np.argmax(poss)]

        polyline = np.array([[point.x, point.y] for point in code.polygon]).reshape((-1, 1, 2))
        if color == "red":
            cv2.polylines(frame, [polyline], True, (0, 0, 255), 3)
            cv2.putText(frame, f"red: {poss[np.argmax(poss)]:.2f}", (left, top), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 0, 255), 2)
        if color == "yellow":
            cv2.polylines(frame, [polyline], True, (0, 255, 255), 3)
            cv2.putText(frame, f"yellow: {poss[np.argmax(poss)]:.2f}", (left, top), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)
        if color == "green":
            cv2.polylines(frame, [polyline], True, (0, 255, 0), 3)
            cv2.putText(frame, f"green: {poss[np.argmax(poss)]:.2f}", (left, top), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)
    return frame, color
