#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from mr_voice.msg import Voice


class Respeaker:
    def __init__(self, voice_topic="/voice/text", say_topic="/speaker/say", audio_path_topic="/respeaker/audio_path"):
        self.voice_topic = voice_topic
        self.say_topic = say_topic
        self.audio_path_topic = audio_path_topic
        
        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=10)
        
        rospy.Subscriber(self.voice_topic, Voice, callback=self.__text_callback)
        rospy.Subscriber(self.audio_path_topic, String, callback=self.__audio_path_callback)
        
        self.voice = [Voice(), ""]

    def __text_callback(self, msg):
        self.voice[0] = msg

    def __audio_path_callback(self, msg):
        self.voice[1] = msg

    def get_voice(self):
        voice = self.voice[0]
        self.voice = [Voice(), ""]
        return voice

    def get_text(self):
        text = self.voice[0].text
        self.voice = [Voice(), ""]
        return text

    def get_path(self):
        path = self.voice[1]
        self.voice = [Voice(), ""]
        return path
    
    def say(self, text):
        self.say_pub.publish(text)


# usage
if __name__ == "__main__":
    rospy.init_node("asdf")
    rospy.loginfo("Node started")
    respeaker = Respeaker()
    while not rospy.is_shutdown():
        text = respeaker.get_text()
        if text != "":
            print(repr(text))
        rospy.Rate(20).sleep() # very important

