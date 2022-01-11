#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from mr_voice.msg import Voice


class Respeaker:
    def __init__(self, voice_topic="/voice/text", say_topic="/speaker/say"):
        self.voice_topic = voice_topic
        self.say_topic = say_topic

        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=10)
        
        rospy.Subscriber(self.voice_topic, Voice, callback=self.__text_callback)
        
        self.voice = Voice()
    
    def __text_callback(self, msg):
        self.voice = msg
    
    def get_voice(self):
        voice = self.voice.copy()
        self.voice = Voice()
        return voice
    
    def get_text(self):
        text = self.voice.text
        self.voice = Voice()
        return text
    
    def say(self, text):
        self.say_pub.publish(text)

