#!/usr/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import Servo
from camera_detect import detect_green_in_image

def detect_angle_with_green():
    rospy.init_node('servo_to_detect_green')
    
    angles = [-90, -60, -30, 0, 30, 60, 90]
    set_servo = rospy.ServiceProxy('/servo_controller/set_position', Servo)

    traj = JointTrajectory()

    for angle_deg in angles:
        try:
            point = JointTrajectoryPoint()

            point.positions = [math.radians(angle_deg)]
            traj.points.append(point)
            
            rospy.loginfo(f"Enviando requisição para mover o servo para {angle_deg}°...")
            response = set_servo(traj)
        
            rospy.loginfo(f"Resposta: {response.message} (Sucesso: {response.success}, Ângulo: {response.angle:.2f}°)")

            if detect_green_in_image():
                rospy.loginfo(f"Verde detectado em {angle_deg}°")

                point = JointTrajectoryPoint()
                point.positions = [0]
                traj.points.append(point)
                set_servo(traj)
                
                return angle_deg 
            else: 
                rospy.loginfo(f"Verde não detectado")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço: {e}")

if __name__ == "__main__":
    try:
        detect_angle_with_green()
    except rospy.ROSInterruptException:
        pass