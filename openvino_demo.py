#!/usr/bin/env python3
from openvino.runtime import Core
import cv2
import rospy
from sensor_msgs.msg import Image

def callback_image(msg):
    global frame
    CvBridge().imgmsg_to_cv2(msg, "bgr8")

# Loading Model
ie = Core()
model_xml = "/home/pcms/models/openvino/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.xml"
model_bin = "/home/pcms/models/openvino/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.bin"
read_model = ie.read_model(model=model_xml)
compiled_model = ie.compile_model(model=read_model, device_name="CPU")
input_layer_ir = next(iter(compiled_model.inputs))

# Other ros parameters
frame = None
rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)

rospy.init_node("openvino_deno")
rospy.loginfo("Test Node Started")

net = cv2.dnn.readNetFromModelOptimizer(model_xml, model_bin)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
net.setPreferable.Target(cv2.DNN_TARGET_CPU)

while not rospy.is_shutdown():
    rospy.Rate(20).sleep()
    if frame is None: continue
    
    # Resizing
   
    out_blob = cv2.dnn.blobFromImage(
        frame,
        scalefactpr=1.0
        size=image.shape[:2],
        mean=(0, 0, 0)
        swapRB=False,
        crop=False
    )
    net.setInput(out_blob)
    res = net.forward()
    
    
