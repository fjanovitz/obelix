#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math
from maze.srv import Finder

class ReactiveNavigator:
    def __init__(self):
        rospy.init_node('reactive_navigator')

        # --- Parâmetros ---
        self.forward_speed = 1.0          # Velocidade máxima (m/s)
        self.turn_speed = math.pi         # Giro máximo (rad/s)
        self.goal_direction = 0.0         # 0° = frente (Y+), -90° = esquerda (X-), 90° = direita (X+)
        self.obstacle_threshold = 0.20     # Limite de distância para obstáculo (m)
        self.wall_follow_distance = 0.05   # Distância alvo para seguir parede (m)
        self.scan_angle_threshold = 120   # Ângulo de bloqueio para ativar scanning (graus)

        # Parâmetros VFH para varredura de 180°
        self.vfh_num_sectors = 36         # Cobertura de 180° (5° por setor)
        self.vfh_valley_threshold = 5     # Mínimo de setores livres consecutivos
        self.vfh_robot_width_sectors = 3  # Largura do robô em setores
        self.vfh_target_weight = 5.0      # Peso da direção alvo
        self.vfh_width_weight = 1.0       # Peso da largura do vale

        # Ganhos do controlador
        self.steer_gain = 1.0
        self.wall_follow_kp = 10.0

        # Máquina de estados
        self.state = 'NAVIGATING'
        self.latest_polar_histogram = np.full(self.vfh_num_sectors, float('inf'))
        self.lidar_ready = False
        self.current_twist = Twist()

        # --- Configuração ROS ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lidar_points', PointCloud2, self.lidar_callback, queue_size=1)

        # Cliente de serviço
        rospy.loginfo("Aguardando serviço de busca de alvo...")
        try:
            rospy.wait_for_service('/camera_controller/find_target', timeout=15.0)
            self.find_target_service = rospy.ServiceProxy('/camera_controller/find_target', Finder)
            rospy.loginfo("Serviço conectado.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"Falha na conexão do serviço: {e}")
            rospy.signal_shutdown("Serviço indisponível")
            return

        rospy.loginfo("Navegador pronto. Iniciando no estado NAVIGATING.")

    def lidar_callback(self, msg):
        """Processa dados do LIDAR com Y+ como frente, X- como esquerda, X+ como direita"""
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))
            if points:
                self.update_vfh_histogram(points)
                self.lidar_ready = True
        except Exception as e:
            rospy.logerr(f"Erro no callback do LIDAR: {e}")
            self.lidar_ready = False

    def update_vfh_histogram(self, points):
        """Cria histograma para varredura de 180° (frente e laterais)"""
        histogram = np.full(self.vfh_num_sectors, float('inf'))
        sector_angle = 180.0 / self.vfh_num_sectors

        for x, y in points:
            dist = math.hypot(x, y)
            if dist < 0.12 or dist > 3.0:  # Faixa do LIDAR
                continue

            # Calcula ângulo com Y+ como frente (0°)
            angle_rad = math.atan2(x, y)
            angle_deg = math.degrees(angle_rad)

            # Converte para faixa de 0-180° (-90° a +90°)
            if -90 <= angle_deg <= 90:
                sector_index = int((angle_deg + 90) / sector_angle)
                sector_index = min(sector_index, self.vfh_num_sectors - 1)

                if dist < histogram[sector_index]:
                    histogram[sector_index] = dist

        histogram[~np.isfinite(histogram)] = 3.0  # Distância máxima
        self.latest_polar_histogram = histogram

    def check_wide_blockage(self):
        """Verifica se há um bloqueio amplo à frente (parede)"""
        # Ângulo de varredura para verificar bloqueio (centrado em 0°)
        scan_range = self.scan_angle_threshold / 2
        start_angle = -scan_range
        end_angle = scan_range

        # Converte para índices do histograma
        start_index = int(((start_angle + 90) / (180.0/self.vfh_num_sectors)))
        end_index = int(((end_angle + 90) / (180.0/self.vfh_num_sectors)))

        # Garante que os índices estejam dentro dos limites
        start_index %= self.vfh_num_sectors
        end_index %= self.vfh_num_sectors

        # Verifica setores no intervalo (considerando a circularidade do histograma)
        blocked_sectors = 0
        if start_index <= end_index:
            for i in range(start_index, end_index + 1):
                if self.latest_polar_histogram[i] < self.obstacle_threshold:
                    blocked_sectors += 1
            total_sectors = end_index - start_index + 1
        else:  # O intervalo cruza o limite (0/35)
            for i in range(start_index, self.vfh_num_sectors):
                if self.latest_polar_histogram[i] < self.obstacle_threshold:
                    blocked_sectors += 1
            for i in range(0, end_index + 1):
                if self.latest_polar_histogram[i] < self.obstacle_threshold:
                    blocked_sectors += 1
            total_sectors = (self.vfh_num_sectors - start_index) + (end_index + 1)

        # Se mais de 80% dos setores estão bloqueados, considera como parede
        return (blocked_sectors / total_sectors) > 0.8 if total_sectors > 0 else False

    def find_best_valley(self):
        """Encontra o melhor caminho na varredura de 180°"""
        if not self.lidar_ready:
            rospy.logwarn_throttle(5, "Aguardando dados do LIDAR...")
            return None

        # Histograma binário (1 = bloqueado)
        binary_hist = (self.latest_polar_histogram < self.obstacle_threshold).astype(int)

        # Aplica máscara da largura do robô
        masked_hist = np.copy(binary_hist)
        blocked_indices = np.where(binary_hist == 1)[0]
        for idx in blocked_indices:
            for offset in range(-self.vfh_robot_width_sectors, self.vfh_robot_width_sectors + 1):
                masked_hist[(idx + offset) % self.vfh_num_sectors] = 1

        # Encontra vales de espaço livre
        valleys = []
        in_valley = False
        start_index = -1

        extended_hist = np.concatenate((masked_hist, masked_hist))

        for i in range(len(extended_hist)):
            current_index = i % self.vfh_num_sectors
            if extended_hist[i] == 0 and not in_valley:
                in_valley = True
                start_index = current_index
            elif (extended_hist[i] == 1 or i == len(extended_hist)-1) and in_valley:
                in_valley = False
                end_index = (i-1) % self.vfh_num_sectors

                width = (end_index - start_index + 1) % self.vfh_num_sectors
                if width >= self.vfh_valley_threshold:
                    center_index = (start_index + end_index) / 2
                    center_angle = center_index * (180.0/self.vfh_num_sectors) - 90.0
                    valleys.append({
                        'center_angle': center_angle,
                        'width': width,
                        'start': start_index,
                        'end': end_index
                    })

        if not valleys:
            rospy.logwarn("Nenhum vale adequado encontrado.")
            return None

        # Seleciona o melhor vale
        best_valley = min(valleys, key=lambda v:
            (self.vfh_target_weight * min(abs(v['center_angle'] - self.goal_direction),
                                          360 - abs(v['center_angle'] - self.goal_direction)) +
            (self.vfh_width_weight * (self.vfh_num_sectors / v['width']))))

        return best_valley['center_angle']

    def execute_scan_sequence(self):
        """Comportamento de varredura com tratamento melhorado do alvo"""
        rospy.loginfo("Iniciando sequência de varredura")
        self.current_twist = Twist()
        rospy.sleep(0.5)  # Pequena pausa para estabilizar

        # try:
        response = self.find_target_service()
        if response.found:
            rospy.loginfo(f"Alvo encontrado em {response.angle_degrees:.2f}°")
            self.goal_direction = response.angle_degrees
            self.state = 'NAVIGATING'
            # # Se estiver muito próximo, para completamente
            # if response.distance < 0.5:
            #     rospy.loginfo("Alvo muito próximo - mantendo parado")
            #     self.state = 'STOP'  # Mas não se move (TEORICAMENTE ENCONTROU O ALVO)
            # else:
            #     self.state = 'NAVIGATING'  # Navega até o alvo
        else:
            rospy.loginfo("Alvo não encontrado - girando")
            self.state = 'ROTATE'
        # except Exception as e:
        #     rospy.logerr(f"Falha na varredura: {e}")
        #     self.state = 'NAVIGATION'

    def execute_navigation(self):
        """Navegação melhorada com desvio inteligente de obstáculos"""
        # Verifica se há uma parede frontal (bloqueio amplo)
        if self.check_wide_blockage():
            rospy.loginfo("Parede frontal detectada - entrando em SCANNING")
            self.state = 'SCANNING'
            self.current_twist = Twist()
            return

        best_angle = self.find_best_valley()

        if best_angle is None:
            rospy.logwarn("Nenhum caminho encontrado, entrando em SCANNING")
            self.state = 'SCANNING'
            self.current_twist = Twist()
            return

        # Verifica obstáculos frontais (-30° a 30°)
        front_sectors = range(
            int(((-30 + 90) / (180.0/self.vfh_num_sectors))),
            int(((30 + 90) / (180.0/self.vfh_num_sectors))) + 1
        )
        front_dists = self.latest_polar_histogram[list(np.mod(front_sectors, self.vfh_num_sectors))] # Handle wrap-around
        front_dist = np.min(front_dists[np.isfinite(front_dists)]) if np.any(np.isfinite(front_dists)) else float('inf')

        # Caso especial: obstáculo frontal muito próximo
        if front_dist < self.obstacle_threshold * 1.5:
            if(front_dist < self.obstacle_threshold):
                self.state = 'SCANNING'
                return
            # Verifica qual lado tem mais espaço
            left_sectors = range(
                int(((-90 + 90) / (180.0/self.vfh_num_sectors))),
                int(((-30 + 90) / (180.0/self.vfh_num_sectors)))
            )
            right_sectors = range(
                int(((30 + 90) / (180.0/self.vfh_num_sectors))),
                int(((90 + 90) / (180.0/self.vfh_num_sectors)))
            )

            left_dists = self.latest_polar_histogram[list(np.mod(left_sectors, self.vfh_num_sectors))]
            right_dists = self.latest_polar_histogram[list(np.mod(right_sectors, self.vfh_num_sectors))]

            left_dist = np.min(left_dists[np.isfinite(left_dists)]) if np.any(np.isfinite(left_dists)) else float('inf')
            right_dist = np.min(right_dists[np.isfinite(right_dists)]) if np.any(np.isfinite(right_dists)) else float('inf')

            rospy.loginfo(f"Obstáculo frontal - Distâncias: Esquerda={left_dist:.2f}m, Direita={right_dist:.2f}m")

            # Decide direção de giro baseado no espaço disponível
            if left_dist > right_dist:
                rospy.loginfo("Obstáculo frontal - virando para DIREITA (mais espaço à esquerda)")
                self.current_twist.linear.x = 0.2 # Reduz a velocidade ao virar
                self.current_twist.angular.z = -self.turn_speed * 0.6  # Negativo para direita
            else:
                rospy.loginfo("Obstáculo frontal - virando para ESQUERDA (mais espaço à direita)")
                self.current_twist.linear.x = 0.2 # Reduz a velocidade ao virar
                self.current_twist.angular.z = self.turn_speed * 0.6  # Positivo para esquerda
            return

        # Calcula comando de direção com desvio de obstáculos melhorado
        angle_error_rad = math.radians(best_angle)

        # Aplica giro mais agressivo quando perto de obstáculos
        best_angle_index = int((best_angle + 90) / (180.0/self.vfh_num_sectors)) % self.vfh_num_sectors
        clearance = self.latest_polar_histogram[best_angle_index]
        turn_gain = self.steer_gain * (1.0 + (1.0 - min(1.0, clearance/self.obstacle_threshold)))

        turn_command = turn_gain * angle_error_rad
        turn_command = np.clip(turn_command, -self.turn_speed, self.turn_speed)

        # Controle de velocidade adaptativo
        speed_scale = max(0.1, 1.0 - abs(turn_command)/self.turn_speed)
        forward_command = self.forward_speed * speed_scale

        # Reduz velocidade baseado na proximidade
        if clearance < self.obstacle_threshold * 2.0:
            forward_command *= (clearance / (self.obstacle_threshold * 2.0))

        self.current_twist.linear.x = max(0.05, forward_command)
        self.current_twist.angular.z = turn_command

        # Comportamento de seguir parede quando perto de obstáculos e com ângulo significativo
        if abs(best_angle) > 30:  # Se estiver significativamente virado
            side_angle_index = int(((best_angle + 90) / (180.0/self.vfh_num_sectors))) % self.vfh_num_sectors
            side_dist = self.latest_polar_histogram[side_angle_index]
            if side_dist < self.wall_follow_distance * 1.5:
                # Determine se a parede está à esquerda ou direita (aproximado)
                if best_angle > 0: # Tendendo para a direita, parede à esquerda
                    error = self.wall_follow_distance - side_dist
                    turn_cmd = self.wall_follow_kp * error
                    rospy.loginfo(f"Ajuste de seguir parede ESQUERDA: {turn_cmd:.2f}")
                    self.current_twist.angular.z = np.clip(
                        self.current_twist.angular.z + turn_cmd,
                        -self.turn_speed,
                        self.turn_speed
                    )
                else: # Tendendo para a esquerda, parede à direita
                    error = side_dist - self.wall_follow_distance
                    turn_cmd = -self.wall_follow_kp * error
                    rospy.loginfo(f"Ajuste de seguir parede DIREITA: {turn_cmd:.2f}")
                    self.current_twist.angular.z = np.clip(
                        self.current_twist.angular.z + turn_cmd,
                        -self.turn_speed,
                        self.turn_speed
                    )

        rospy.loginfo(f"NAV: Frente:{forward_command:.2f}m/s, Giro:{turn_command:.2f}rad/s, Distância:{clearance:.2f}m, Best Angle:{best_angle:.2f}°")

    def execute_rotation(self, clockwise=True):
        rospy.loginfo("Executando rotação de 90 graus")
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -self.turn_speed if clockwise else self.turn_speed

        rotation_duration = math.pi / 2 / self.turn_speed  # 90 graus / velocidade
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz

        while rospy.Time.now() - start_time < rospy.Duration(rotation_duration):
            self.cmd_pub.publish(self.current_twist)
            rate.sleep()

        # Parar movimento após a rotação
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)

        rospy.loginfo("Rotação concluída. Mudando para NAVIGATING")
        self.state = 'NAVIGATING'


    def run(self):
        """Loop principal de controle"""
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown() and not self.lidar_ready:
            rospy.loginfo_throttle(1, "Aguardando LIDAR...")
            rate.sleep()

        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.execute_navigation()
            elif self.state == 'SCANNING':
                self.execute_scan_sequence()
                continue
            elif self.state == 'ROTATE':
                self.execute_rotation(True)
            elif self.state == 'STOP':
                self.current_twist = Twist()
            self.cmd_pub.publish(self.current_twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        nav = ReactiveNavigator()
        nav.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Desligamento solicitado - parando...")
        stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        for _ in range(10):
            stop_pub.publish(Twist())
            rospy.sleep(0.1)
    except Exception as e:
        rospy.logerr(f"Erro: {e}")
    finally:
        rospy.Publisher('/cmd_vel', Twist, queue_size=1).publish(Twist())