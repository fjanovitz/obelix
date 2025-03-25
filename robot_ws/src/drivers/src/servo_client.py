#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import SetServo

def send_servo_request(angle_deg):
    rospy.init_node('test_servo_client')
    rospy.wait_for_service('/controlador_servo/set_position')

    try:
        set_servo = rospy.ServiceProxy('/controlador_servo/set_position', SetServo)
        
        # Cria a mensagem JointTrajectory
        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [angle_deg * 3.1416 / 180]  # Converte para radianos
        traj.points.append(point)

        response = set_servo(traj)
        rospy.loginfo(f"Resposta: {response.message} (Sucesso: {response.success}, Ângulo: {response.angle:.2f}°)")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro no serviço: {e}")

if __name__ == '__main__':
    send_servo_request(45)  # Altere o ângulo conforme quiser
