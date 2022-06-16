#!/usr/bin/env python3
import rospy
import core


rospy.init_node("home_service_robot")

assistant = core.nlu.Assistant()
tar_name = "assistant.tar"
retrain_nlu = True
if retrain_nlu:
    assistant.set_dataset(core.nlu.Dataset().from_yaml("dataset.yaml"))
    assistant.save(tar_name)
else:
    assistant.load(tar_name)

respond = core.nlu.Respond(assistant)
respond.from_yaml("respond.yaml")

respeaker = core.Respeaker()

rospy.loginfo("ready")
while not rospy.is_shutdown():
    rospy.Rate(20).sleep()

    text = respeaker.get_text()
    if text != "":
        print(f"Text: {text}")
        for res in respond.get_respond(text):
            if res["type"] == "say":
                print(f"Robot: {res['text']}")
                respeaker.say(res["text"])
