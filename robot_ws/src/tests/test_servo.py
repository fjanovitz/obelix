#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import Servo
import math

def set_servo_position(angle_degrees):
    rospy.wait_for_service('servo_controller/set_position')
    try:
        # Converte graus para radianos (que é o que o serviço espera)
        angle_radians = math.radians(angle_degrees)
        
        # Cria a mensagem de trajetória
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [angle_radians]
        trajectory.points.append(point)
        
        # Chama o serviço
        servo_proxy = rospy.ServiceProxy('servo_controller/set_position', Servo)
        response = servo_proxy(trajectory)
        
        print(f"Resposta do serviço: {response.message}")
        print(f"Ângulo atual: {response.angle:.2f}°")
        return response.success
        
    except rospy.ServiceException as e:
        print(f"Falha ao chamar o serviço: {e}")
        return False

def test_sequence():
    # Testa uma sequência de movimentos
    angles = [-90, -45, 0, 45, 90, 0]  # Sequência de ângulos para testar
    for angle in angles:
        print(f"\nMovendo para {angle}°")
        success = set_servo_position(angle)
        if not success:
            print("Movimento falhou, abortando teste")
            return
        rospy.sleep(1)  # Espera 1 segundo entre movimentos

if __name__ == '__main__':
    rospy.init_node('servo_client')
    
    print("\nCliente de teste do Servo Controller")
    print("----------------------------------")
    
    while not rospy.is_shutdown():
        print("\nOpções:")
        print("1. Enviar ângulo específico")
        print("2. Executar sequência de teste")
        print("3. Sair")
        
        choice = input("Escolha uma opção (1-3): ")
        
        if choice == '1':
            try:
                angle = float(input("Digite o ângulo (-90 a 90): "))
                set_servo_position(angle)
            except ValueError:
                print("Entrada inválida. Digite um número.")
        elif choice == '2':
            test_sequence()
        elif choice == '3':
            break
        else:
            print("Opção inválida. Tente novamente.")
    
    print("Encerrando cliente do servo.")