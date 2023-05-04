#!/usr/bin/env python3
import torch
from torch import nn
from torch.utils.data import DataLoader

import matplotlib.pyplot as plt

from model import Model
from dataset import HealthCodes


continue_training = True

device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"using device: {device}")

model = Model()
if continue_training:
    model.load_state_dict(torch.load('model-1.pth'))
model.to(device)

dataset = HealthCodes()
dataloader = DataLoader(
    dataset,
    batch_size=50,
    shuffle=True
)

loss_fn = nn.MSELoss()
optimizer = torch.optim.Adam(
    model.parameters(),
    lr=1e-3
)


epochs = 100
model.train()
for epoch in range(epochs):
    for batch, (x, y) in enumerate(dataloader):
        x = torch.stack([i.to(device) for i in x])
        y = torch.stack([i.to(device) for i in y])

        h = model(x)
        loss = loss_fn(h, y)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(f"Epoch: {epoch} ({batch:03d} / {len(dataloader)}): train loss: {loss.item():.4f}")
    torch.save(model.state_dict(), 'model-3.pth')
