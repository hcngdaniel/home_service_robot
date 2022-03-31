#!/usr/bin/env python3
import rospy
import core


rospy.init_node("assistant_with_respeaker_demo")
respeaker = core.Respeaker()
tts = core.YourTTS("Kinda.wav")


def callback(session):
    if session.intent_name == "turnLightOn":
        while "room" in session.missing_slots:
            tts.say("Okay, but which room?")
            try:
                session.set_slot("room", input())
            except ValueError:
                tts.say("sorry, I cannot understand")
        tts.say(f"Okay, I will turn on the lights in {session.parse_result['slots'][0]['value']['value']}")
    elif session.intent_name == "turnLightOff":
        while "room" in session.missing_slots:
            tts.say("Okay, but which room?")
            try:
                session.set_slot("room", input())
            except ValueError:
                tts.say("sorry, I cannot understand")
        tts.say(f"Okay, I will turn off the lights in {session.parse_result['slots'][0]['value']['value']}")
    elif session.intent_name is None:
        tts.say("sorry, I cannot understand")


assistant = core.nlu.Assistant()
assistant.set_dataset(core.nlu.Dataset().from_yaml("core/nlu/example_dataset.yaml"))
assistant.save("asdf.tar")
print("ready")
while not rospy.is_shutdown():
    text = respeaker.get_text()
    if text != "":
        print(text)
        assistant.session.request(text, callback=callback)
    rospy.Rate(20).sleep()
