#!/usr/bin/env python3
import core
import rospy
import time


rospy.init_node("find_my_mates")

up_cam = core.Astra("ucam")
down_cam = core.Astra("dcam")
chassis = core.Chassis()
respeaker = core.Respeaker()
assistant = core.nlu.Assistant.load("cml_assistant.ass")
session = assistant.session
navigation = core.Navigation()
manipulator = core.Manipulator()
PA100k_PAR = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR")
PETA_PAR = core.openvino_models.resnet50_PAR(model_name="resnet50_PAR_PETA")

time.sleep(3)


