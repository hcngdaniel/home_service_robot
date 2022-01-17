#!/usr/bin/env python3
from core import Assistant, Dataset, Session


def callback(session):
    if session.intent_name == "turnLightOn":
        while "room" in session.missing_slots:
            print("Ok, but which room?")
            try:
                session.set_slot("room", input())
            except Exception:
                print("sorry, I cannot understand")
        print(f"Ok, I will turn on the lights in {session.parse_result['slots'][0]['value']['value']}")
    elif session.intent_name == "turnLightOff":
        while "room" in session.missing_slots:
            print("Ok, but which room?")
            try:
                session.set_slot("room", input())
            except Exception:
                print("sorry, I cannot understand")
        print(f"Ok, I will turn off the lights in {session.parse_result['slots'][0]['value']['value']}")
    elif session.intent_name == None:
        print("sorry, I cannot understand")

assistant = Assistant.load("test.tar")
print("ready")
for i in range(1000000):
    assistant.session.request(input(), callback=callback)
