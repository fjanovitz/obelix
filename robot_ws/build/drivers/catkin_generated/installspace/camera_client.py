#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from drivers.srv import CaptureImage
import sys

def capture_image_client():
    rospy.init_node("camera_client")
    
    rospy.loginfo("Esperando o serviço de câmera")
    rospy.wait_for_service("image")
    
    try:
        capture_service = rospy.ServiceProxy("image", CaptureImage)
        
        rospy.loginfo("Requisitando imagem capturada")
        response = capture_service()
        
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
            
            image_path = "image.jpg"
            cv2.imwrite(image_path, cv_image)
            rospy.loginfo(f"Imagem salva como {image_path}")
            
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CV Bridge: {e}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Chamada ao serviço de câmera falhou: {e}")

if __name__ == "__main__":
    try:
        capture_image_client()
    except rospy.ROSInterruptException:
        pass
