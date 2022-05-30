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
    
    last_mid = [0, 0]
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        
        if _frame is None: continue
        if _boxes is None: continue
        
        people_in_frame = 0
        classes = []
        frame = _frame.copy()
        
        
        
        cv2.line(frame, (640 // 3, 0), (640 // 3, 480), (250, 0, 0), 1)
        cv2.line(frame, (640 // 3 * 2, 0), (640 // 3 * 2, 480), (250, 0, 0), 1)
        
        for box in _boxes:
            if box.Class == "person":
                coordinates = []
                xmin, xmax = box.xmin, box.xmax
                ymin, ymax = box.ymin, box.ymax
                cx = (xmin + xmax) // 2
                cy = (ymin + ymax) // 2
                
                cv2.circle(frame, (cx, cy), 5, (0, 0, 0), -1)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                
                
                rospy.loginfo(last_mid)
                
                
                if last_mid[0] < (640 // 3) and cx > (640 // 3):
                    library_person_count += 1
                if last_mid[0] > (640 // 3 * 2) and cx < (640 // 3 * 2):
                    library_person_count -= 1
                
                if library_person_count <= 0: library_person_count = 0
                
                cv2.rectangle(frame, (0, 0), (400, 80), (0, 0, 0), -1)
                cv2.putText(
                    frame, 
                    'Person Count: %d' % library_person_count, 
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    2
                
                )
                
                last_mid = [cx, cy]
                
            classes.append(box.Class)
            
            
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)
        rospy.loginfo("Person Count:  %d" % classes.count("person")) 
        
        
            
    rospy.loginfo("Count Node End!")
