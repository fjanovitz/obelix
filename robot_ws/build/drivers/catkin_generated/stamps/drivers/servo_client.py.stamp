#!/usr/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import Servo

if __name__ == '__main__':
    angle_deg = 0
    count = 0
    print(f"Enviando requisição para mover o servo para {angle_deg}°...")# Altere o ângulo conforme quiser
    rospy.init_node('test_servo_client')

    try:
        # Cria a mensagem JointTrajectory
        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle_deg)]
        traj.points.append(point)
        set_servo = rospy.ServiceProxy('/servo_controller/set_position', Servo)
        rospy.loginfo(f"Enviando requisição para mover o servo para {angle_deg}°...")
        response = set_servo(traj)
        rospy.loginfo(f"Resposta: {response.message} (Sucesso: {response.success}, Ângulo: {response.angle:.2f}°)")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro no serviço: {e}")
