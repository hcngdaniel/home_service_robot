#!/usr/bin/env python3
from core import Assistant, Dataset, Session


def callback(session):
    if session.intent_name == "turnLightOn":
        while "room" in session.missing_slots:
            print("Ok, but which room?")
            session.set_slot("room", input())
        print(f"Ok, I will turn on the lights in {session.parse_result['slots'][0]['value']['value']}")
    elif session.intent_name == "turnLightOff":
        while "room" in session.missing_slots:
            print("Ok, but which room?")
            session.set_slot("room", input())
        print(f"Ok, I will turn off the lights in {session.parse_result['slots'][0]['value']['value']}")

assistant = Assistant.load("test.tar")
print("ready")
for i in range(100):
    assistant.session.request(input(), callback=callback)
