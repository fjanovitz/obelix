#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ydlidar
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

WINDOW_SIZE = 5

class LidarPublisher:
    def __init__(self):
        rospy.init_node("lidar_publisher", anonymous=True)
        pub = rospy.Publisher("/lidar_points", PointCloud2, queue_size=10)

        laser = self.setup_lidar()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            points = self.get_lidar_data(laser)
            
            if points:
                header = rospy.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "laser_frame"

                # Cria mensagem PointCloud2
                cloud_msg = pc2.create_cloud_xyz32(header, points)
                pub.publish(cloud_msg)

            rate.sleep()

    def smooth_data(self, points):
        # Aplica um filtro de média móvel para suavizar os dados.
        if len(points) < WINDOW_SIZE:
            return points  # Se poucos pontos, retorna como está

        smoothed = []
        for i in range(len(points) - WINDOW_SIZE + 1):
            avg_x = sum(p[0] for p in points[i:i+WINDOW_SIZE]) / WINDOW_SIZE
            avg_y = sum(p[1] for p in points[i:i+WINDOW_SIZE]) / WINDOW_SIZE
            smoothed.append((avg_x, avg_y, 0.0))
        return smoothed

    def polar_to_cartesian(self, points):
        # Converte coordenadas polares (ângulo, distância) em cartesianas (X, Y). 
        cartesian_points = []
        for p in points:
            if 0.12 <= p.range <= 6.0:  # Filtrando distâncias válidas
                x = p.range * math.cos(p.angle + math.radians(32))
                y = p.range * math.sin(p.angle + math.radians(32))
                cartesian_points.append((x, y, 0.0))
        return cartesian_points

    def setup_lidar(self):
        ydlidar.os_init()
        laser = ydlidar.CYdLidar()
        ports = ydlidar.lidarPortList()
        
        # Detectando a porta do LiDAR
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            print(f"LiDAR encontrado na porta: {port}")

        # Configuração do LiDAR
        laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)

        # Ajustes de varredura
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        laser.setlidaropt(ydlidar.LidarPropMaxAngle, -60.0)
        laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
        laser.setlidaropt(ydlidar.LidarPropMaxRange, 6.0)
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

        if not laser.initialize():
            raise RuntimeError("Falha ao inicializar o YDLidar!")

        if not laser.turnOn():
            raise RuntimeError("Falha ao ligar o YDLidar!")

        return laser

    def get_lidar_data(self, laser):
        scan = ydlidar.LaserScan()
        r = laser.doProcessSimple(scan)
        if r:
            num_points = len(scan.points)
            scan_freq = 1.0 / scan.config.scan_time
            print(f"Scan recebido [{scan.stamp}]: {num_points} pontos | {scan_freq:.2f} Hz")

            # Conversão para coordenadas cartesianas
            cartesian_data = self.polar_to_cartesian(scan.points)

            # Aplicando filtro de suavização
            # smoothed_data = smooth_data(cartesian_data)

            # Exibindo apenas os primeiros 10 pontos filtrados e suavizados
            print("Pontos Processados:")
            for x, y, z in cartesian_data:
                print(f"X: {x:.2f} m | Y: {y:.2f} m")

            return cartesian_data
        return []

if __name__ == "__main__":
    try:
        lidar = LidarPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro no serviço: {e}")
