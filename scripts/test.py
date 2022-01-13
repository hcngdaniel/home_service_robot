#!/usr/bin/env python3
from core import Assistant, Dataset

assitant = Assistant(0.2)
dataset = Dataset().from_yaml("../NLU_files/test.yaml")
assitant.set_dataset(dataset)
assitant.set_response("turnLightOn", "Ok, I will turn the lights on in {room.value.value}")
print(assitant("Hey, lights on in the lounge!"))
