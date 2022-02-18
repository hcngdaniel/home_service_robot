#!/usr/bin/env python3
import sys
import os
import rospkg
import rospy

try:
    sys.path.append(f"{rospkg.RosPack().get_path('home_service_robot')}/scripts")
except:
    sys.path.append("/home/hcng/catkin_ws/src/home_service_robot/scripts")

import core


if __name__ == '__main__':
    rospy.init_node('find_book_demo')

astra = core.Astra()
navigation = core.Navigation()
respeaker = core.Respeaker()
assistant = core.nlu.Assistant.load(f'{os.path.dirname(__file__)}/assistant.assistant')
tts = core.YourTTS(['/home/hcng/catkin_ws/src/home_service_robot/scripts/sample.wav'])


def find_book(text: str):
    pass


if __name__ == '__main__':
    find_book('')
