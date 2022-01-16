#!/usr/bin/env python3
from core import Assistant, Dataset, Session


def callback(intent, missing_slots, response, assistant):
    session = Session(assistant)
    session.request(intent["input"])
    session.set_slot("room", "please turn on the lights in kitchen")
    print(session.parse_result)
    s0 = "slots"
    s1 = "value"
    print(f"Session: Ok, I will turn on the lights in {session.parse_result[s0][0][s1][s1]}")

assistant = Assistant.load("test.tar")
assistant.set_dataset(Dataset().from_yaml("../NLU_files/test.yaml"))
assistant.set_response("turnLightOn", "Ok, I will turn the lights on in {room.value.value}", callback=callback)
assistant.set_response("SLOTroom", "u mean the slot {room.value.value}")
print("ready")
for i in range(100):
    print(assistant(input()))
