#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class Respeaker:
    def __init__(self, text_topic="/voice/text", say_topic="/speaker/say"):
        self.text_topic = text_topic
        self.say_topic = say_topic

        self.say_pub = rospy.Publisher(say_topic, String, queue_size=10)
        
        rospy.Subcriber(text_topic, String, callback=self.__text_callback)
        rospy.wait_for_message(text_topic, String)
        self.text = ""
    
    def __text_callback(self, msg):
        self.text = msg.data
    
    def text(self):
        return self.text
    
    def say(self, text):
        self.say_pub.publish(text)

