#!/usr/bin/env python3
import rospy
import core
import cv2


tts = core.YourTTS('sample.wav')
tts.say("Hello, I am a respeaker")

exit()
rospy.init_node("test2")
astra = core.Astra()

while not rospy.is_shutdown():
    depth_img = astra.read_depth()
    cv2.imshow("frame", depth_img)
    if cv2.waitKey(16) in [27, ord('q')]:
        break
    print(astra.get_euclidean_distance(320, 240, depth_img))
