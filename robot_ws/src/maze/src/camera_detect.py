#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from drivers.srv import Camera
import numpy as np

def detect_green_in_image():
    rospy.init_node("detect_green_in_image")
    bridge = CvBridge()
    
    rospy.wait_for_service("image")
    capture_service = rospy.ServiceProxy("image", Camera)
    
    try:
        response = capture_service()
        cv_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
        
        kernal = np.ones((5, 5), "uint8")
                               
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        green_mask = cv2.dilate(green_mask, kernal)
        
        if cv2.countNonZero(green_mask) > 0:
            return True
        else:
            return False
        
    except (rospy.ServiceException, CvBridgeError) as e:
        rospy.logerr(f"Erro: {e}")

if __name__ == "__main__":
    try:
        detect_green_in_image()
    except rospy.ROSInterruptException:
        pass