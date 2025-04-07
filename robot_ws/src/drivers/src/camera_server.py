#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from drivers.srv import Camera, CameraResponse  # Isso deve ser um serviço definido em outro arquivo
import time
import numpy as np

class CameraServer:
    def __init__(self):
        rospy.init_node("camera_server")
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)
        
        if not self.camera.isOpened():
            rospy.logerr("Não foi possível abrir a câmera")
            return

        time.sleep(2)

        # Configura para liberar a câmera entre capturas
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.service = rospy.Service("image", Camera, self.handle_capture_request)
        rospy.loginfo("Serviço de câmera pronto")

    def handle_capture_request(self, req):
        try:
            # Libera buffers antigos
            for _ in range(3):
                self.camera.grab()
            
            ret, frame = self.camera.read()
            
            if not ret:
                rospy.logerr("Falha ao capturar frame")
                return CameraResponse()
            
            frame = cv2.rotate(frame, cv2.ROTATE_180)
                
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            return CameraResponse(image=ros_image)
            
        except Exception as e:
            rospy.logerr(f"Erro: {str(e)}")
            return CameraResponse()

    def shutdown(self):
        if self.camera.isOpened():
            self.camera.release()
        rospy.loginfo("Serviço de câmera encerrado")

if __name__ == "__main__":
    try:
        server = CameraServer()  # Use o novo nome da classe
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass