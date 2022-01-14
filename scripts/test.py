#!/usr/bin/env python3
from core import Assistant, Dataset
import io


assistant = Assistant.load("assistant.tar")
# assistant = Assistant(0.2)
# dataset = Dataset()
# dataset = dataset.from_yaml("../NLU_files/test.yaml")
# assistant.set_dataset(dataset)
# assistant.set_response("turnLightOn", "Ok, I will turn the lights on in {room.value.value}")
# assistant.save("assistant.tar")
print(assistant("Please turn the lights on in the bathroom"))
print(assistant("Can you turn on the lights in the lounge"))
print(assistant("Lights on in bedroom"))
