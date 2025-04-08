#!/usr/bin/env python3

import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from drivers.srv import Camera
from drivers.srv import Servo
from maze.srv import Finder, FinderResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
# import os

class TargetFinder:
    def __init__(self):
        rospy.init_node('camera_controller')
        
        self.bridge = CvBridge()
        self.angles = [-90, -60, -30, 0, 30, 60, 90]
        self.green_lower = np.array([25, 52, 72], np.uint8)
        self.green_upper = np.array([102, 255, 255], np.uint8)
        self.min_green_area = 500

        rospy.loginfo("Esperando por serviços da câmera e do servomotor...")
        rospy.wait_for_service("/image")
        rospy.wait_for_service("/servo_controller/set_position")
        rospy.loginfo("Serviços disponíveis.")

        self.capture_service = rospy.ServiceProxy("image", Camera)
        self.set_servo = rospy.ServiceProxy('/servo_controller/set_position', Servo)

        self.service = rospy.Service('find_target', Finder, self.handle_find_target_request)

        self.center_servo()
        rospy.on_shutdown(self.center_servo)

        rospy.loginfo("Serviço de Busca iniciado.")

    def center_servo(self):
        try:
            traj = JointTrajectory()
            point = JointTrajectoryPoint()
            point.positions = [math.radians(0)] # 0 degrees
            traj.points.append(point)
            self.set_servo(traj)
            rospy.loginfo("Servo centralizado.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro em centralizar o servo: {e}")

    def detect_green_in_image(self):
        try:
            response = self.capture_service()
            cv_image = self.bridge.imgmsg_to_cv2(response.image, "bgr8")

            one_third_width = cv_image.shape[1] // 3
            second_third_width = 2 * cv_image.shape[1] // 3
            cv_image = cv_image[:, one_third_width:second_third_width]
            
            kernal = np.ones((5, 5), "uint8")          
            hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            green_mask = cv2.inRange(hsv_frame, self.green_lower, self.green_upper)
            green_mask = cv2.dilate(green_mask, kernal)

            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            total_area = sum(cv2.contourArea(c) for c in contours)

            # timestamp = time.strftime("%Y%m%d_%H%M%S")
            # image_path = f"camera_{timestamp}.jpg"
            # cv2.imwrite(image_path, cv_image)
            # rospy.loginfo(f"Imagem salva como {os.path.abspath(image_path)}")
            
            if  total_area > self.min_green_area:
                rospy.loginfo("O objetivo foi encontrado")
                return True
            else:
                rospy.loginfo("O objetivo não foi encontrado")
                return False
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço: {e}")
        except CvBridgeError as e:
            rospy.logerr(f"Erro no CV Bridge: {e}")

    def handle_find_target_request(self, req):
        rospy.loginfo("Requisição recebida, iniciando busca...")
        found_target = False
        found_angle = 0.0
        for angle_deg in self.angles:
            if rospy.is_shutdown():
                return FinderResponse(found=False, angle_degrees=0.0)

            try:
                traj = JointTrajectory()
                point = JointTrajectoryPoint()
                point.positions = [math.radians(angle_deg)]
                traj.points.append(point)
                
                rospy.loginfo(f"Enviando requisição para mover o servo para {angle_deg}°...")
                response = self.set_servo(traj)

                time.sleep(1)
            
                rospy.loginfo(f"Resposta: {response.message} (Sucesso: {response.success}, Ângulo: {response.angle:.2f}°)")

                if self.detect_green_in_image():
                    rospy.loginfo(f"Verde detectado em {angle_deg}°")
                    found_target = True
                    found_angle = float(angle_deg)
                    break 
                else: 
                    rospy.loginfo(f"Verde não detectado")
                
            except rospy.ServiceException as e:
                rospy.logerr(f"Erro no serviço: {e}")

        self.center_servo()
        rospy.loginfo(f"Escaneamento concluído. Encontrado: {found_target}, Ângulo: {found_angle}")
        return FinderResponse(found=found_target, angle_degrees=found_angle)

if __name__ == "__main__":
    try:
        find = TargetFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro no serviço: {e}")
    