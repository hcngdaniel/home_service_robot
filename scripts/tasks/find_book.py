#!/usr/bin/env python3
import sys
import os
import rospkg
import rospy
import core


if __name__ == '__main__':
    rospy.init_node('find_book_demo')

astra = core.Astra()
navigation = core.Navigation()
respeaker = core.Respeaker()
tts = core.YourTTS(['/home/hcng/catkin_ws/src/home_service_robot/scripts/sample.wav'])
pose_dict = core.utils.PoseDict.load('book_poses.yaml')


def find_book(name: str):
    tts.say(f'Okay, I will bring you to {name}')
    target_book_pose = pose_dict[name.strip()]
    navigation.move_to_pose(target_book_pose)


if __name__ == '__main__':
    find_book('')
