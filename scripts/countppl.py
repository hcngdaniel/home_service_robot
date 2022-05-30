#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from robot_vision_msgs.msg import BoundingBoxes

def callback_image(msg):
    global _frame
    _frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_boxes(msg):
    global _boxes
    _boxes = msg.bounding_boxes


if __name__ == "__main__":
    rospy.init_node("count")
    rospy.loginfo("Count Node Start!")
    
    _frame = None
    rospy.Subscriber("camera/rgb/image_raw", Image, callback_image)
    
    _boxes = []
    rospy.Subscriber("/yolo_ros/bounding_boxes", BoundingBoxes, callback_boxes)
    
    library_person_count = 0
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        
        if _frame is None: continue
        if _boxes is None: continue
        
        people_in_frame = 0
        classes = []
        frame = _frame.copy()
        
        for box in _boxes:
            if box.Class == "person":
                xmin, xmax = box.xmin, box.xmax
                ymin, ymax = box.ymin, box.ymax
                cx = (xmin + xmax) // 2
                cy = (ymin + ymax) // 2
                
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                
                rospy.loginfo("Person Coordinates:")
                rospy.loginfo("XMin: %d, XMax: %d" % (xmin, xmax))
                rospy.loginfo("YMin: %d, YMax: %d" % (ymin, ymax))
                rospy.loginfo("-------------------")
                
            classes.append(box.Class)
            
            
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)
        rospy.loginfo("Person Count:  %d" % classes.count("person"))    
        
            
    rospy.loginfo("Count Node End!")
