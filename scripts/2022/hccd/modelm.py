#!/usr/bin/env python
import torch
from torch import nn


class Model(nn.Module):
    def __init__(self):
        super(Model, self).__init__()
        self.model = nn.Sequential(
            nn.Conv2d(3, 10, (5, 5), stride=(5, 5)),
            nn.ReLU(),
            nn.Conv2d(10, 20, (5, 5), stride=(5, 5)),
            nn.ReLU(),
            nn.Conv2d(20, 40, (1, 1), stride=(1, 1)),
            nn.ReLU(),
            nn.Conv2d(40, 40, (1, 1), stride=(1, 1)),
            nn.ReLU(),
            nn.Conv2d(40, 40, (1, 1), stride=(1, 1)),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(19000, 100),
            nn.ReLU(),
            nn.Linear(100, 3),
            nn.Softmax()
        )

    def forward(self, x):
        return self.model(x)

