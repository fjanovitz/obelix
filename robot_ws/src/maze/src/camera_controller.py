#!/usr/bin/env python3

import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from drivers.srv import Camera
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import Servo
import time
import os

set_servo = rospy.ServiceProxy('/servo_controller/set_position', Servo)

def detect_green_in_image():
    # rospy.init_node("detect_green_in_image")
    bridge = CvBridge()
    
    rospy.wait_for_service("image")
    capture_service = rospy.ServiceProxy("image", Camera)
    
    try:
        response = capture_service()
        cv_image = bridge.imgmsg_to_cv2(response.image, "bgr8")

        one_third_width = cv_image.shape[1] // 3
        second_third_width = 2 * cv_image.shape[1] // 3

        cv_image = cv_image[:, one_third_width:second_third_width]
        
        kernal = np.ones((5, 5), "uint8")
                               
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
        green_mask = cv2.dilate(green_mask, kernal)

        # timestamp = time.strftime("%Y%m%d_%H%M%S")
        # image_path = f"camera_{timestamp}.jpg"
        # cv2.imwrite(image_path, cv_image)
        # rospy.loginfo(f"Imagem salva como {os.path.abspath(image_path)}")
        
        if cv2.countNonZero(green_mask) > 0:
            rospy.loginfo("True")
            return True
        else:
            rospy.loginfo("False")
            return False
        
    except (rospy.ServiceException, CvBridgeError) as e:
        rospy.logerr(f"Erro: {e}")

def find_objective():
    rospy.init_node('objective_finder')
    
    angles = [-90, -60, -30, 0, 30, 60, 90]
    set_servo = rospy.ServiceProxy('/servo_controller/set_position', Servo)

    for angle_deg in angles:
        try:
            traj = JointTrajectory()
            point = JointTrajectoryPoint()

            point.positions = [math.radians(angle_deg)]
            traj.points.append(point)
            
            rospy.loginfo(f"Enviando requisição para mover o servo para {angle_deg}°...")
            response = set_servo(traj)

            time.sleep(1)
        
            rospy.loginfo(f"Resposta: {response.message} (Sucesso: {response.success}, Ângulo: {response.angle:.2f}°)")

            if detect_green_in_image():
                rospy.loginfo(f"Verde detectado em {angle_deg}°")
                return angle_deg 
            else: 
                rospy.loginfo(f"Verde não detectado")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço: {e}")

if __name__ == "__main__":
    try:
        find_objective()
    except rospy.ROSInterruptException:
        pass
    finally:
        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [0]
        traj.points.append(point)
        set_servo(traj)