#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from drivers.srv import Camera
import time
import os

def capture_image_client():
    rospy.init_node("camera_client")
    bridge = CvBridge()
    
    rospy.wait_for_service("image")
    capture_service = rospy.ServiceProxy("image", Camera)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    image_path = f"camera_{timestamp}.jpg"
    
    try:
        response = capture_service()
        cv_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
        
        cv2.imwrite(image_path, cv_image)
        rospy.loginfo(f"Imagem salva como {os.path.abspath(image_path)}")
        
    except (rospy.ServiceException, CvBridgeError) as e:
        rospy.logerr(f"Erro: {e}")

if __name__ == "__main__":
    try:
        capture_image_client()
    except rospy.ROSInterruptException:
        pass