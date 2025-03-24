#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from drivers.srv import CaptureImage, CaptureImageResponse
import time

class Camera:
    def __init__(self):
        rospy.init_node("camera_server")
        
        self.bridge = CvBridge()
        
        self.camera = cv2.VideoCapture(0)

        if not self.camera.isOpened():
            rospy.logerr("Erro: Não foi possível abrir a câmera")
            return
        
        time.sleep(2)
        
        rospy.loginfo("Câmera iniciada")
        
        self.service = rospy.Service("image", CaptureImage, self.handle_capture_request)
        rospy.loginfo("O serviço de câmera está pronto")
    
    def handle_capture_request(self, req):
        rospy.loginfo("Pedido de captura de imagem recebido")
        
        ret, frame = self.camera.read()
        
        if not ret:
            rospy.logerr("Erro: Não foi possível capturar o frame.")
            return CaptureImageResponse()
        
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            return CaptureImageResponse(image=ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CV Bridge: {e}")
            return CaptureImageResponse()
    
    def shutdown(self):
        if self.camera.isOpened():
            self.camera.release()
        rospy.loginfo("Serviço de câmera desligado")

if __name__ == "__main__":
    try:
        camera_service = Camera()
        rospy.on_shutdown(camera_service.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
