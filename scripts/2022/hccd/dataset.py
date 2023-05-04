#!/usr/bin/env python
import os
import cv2
import torch
from torch.utils.data import Dataset
import random
import numpy as np


labels = {
    0: "red",
    1: "yellow",
    2: "green"
}


class HealthCodes(Dataset):
    def __init__(self):
        self.red_codes = [f"data/red/{path}" for path in os.listdir('data/red')]
        self.yellow_codes = [f"data/yellow/{path}" for path in os.listdir('data/yellow')]
        self.green_codes = [f"data/green/{path}" for path in os.listdir('data/green')]
        self.images = self.red_codes + self.yellow_codes + self.green_codes

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        x = cv2.imread(self.images[idx]).astype(np.float32)
        x = x / 255
        x = np.transpose(x, (2, 0, 1))
        torch.as_tensor(x, dtype=torch.float32)
        code = self.images[idx].split("/")[1]
        y = None
        if code == 'red':
            y = torch.as_tensor([1, 0, 0], dtype=torch.float32)
        if code == 'yellow':
            y = torch.as_tensor([0, 1, 0], dtype=torch.float32)
        if code == 'green':
            y = torch.as_tensor([0, 0, 1], dtype=torch.float32)
        return x, y
