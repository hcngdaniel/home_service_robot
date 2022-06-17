#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import torch
from pytorch_models import *
from std_msgs.msg import Int32

def callback_image(msg):
    global _frame
    _frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")



COCO_CLASSES = (
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
)

if __name__ == "__main__":
    rospy.init_node("Person_Countt")
    rospy.loginfo("Person Count Node")

    # Ros Messages

    _frame = None
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    
    # Initalizing Models
    rcnn = FasterRCNN()
    previous_x = 0
    
    people_count = 0
    last_frame = []
    
    pub_frame = rospy.Publisher("frame_for_person_count", Image, queue_size=10)
    pub_count = rospy.Publisher("personcnt", Int32, queue_size=10)
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if _frame is None: continue
        
        frame = _frame.copy()
        frame = cv2.resize(frame, (1280, 960))
        
        # Visualize Lines
        cv2.line(
            frame, 
            (frame.shape[1] // 3, 0), 
            (frame.shape[1] // 3, frame.shape[0]), 
            (255, 255, 0),
            2
        )
        
        cv2.line(
            frame,
            (frame.shape[1] // 3 * 2, 0),
            (frame.shape[1] // 3 * 2, frame.shape[0]),
            (255, 255, 0),
            2
        )
        
        
        boxes = rcnn.forward(frame)
        for _id, index, conf, x1, y1, x2, y2 in boxes:
            if rcnn.labels[index] == "person":
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, rcnn.labels[index], (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1) 
                
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                if previous_x < frame.shape[1] // 2 and cx > frame.shape[1] // 2:
                    people_count += 1
                if previous_x > frame.shape[1] // 2 and cx < frame.shape[1] // 2:
                    people_count -= 1
                
                if people_count < 0: people_count = 0                
                previous_x = cx
        cv2.putText(frame, str(people_count), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 3)
        pub_frame.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))
        pub_count.publish(people_count)
            
        
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) in [27, ord('q')]:
            break


