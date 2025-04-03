#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import numpy as np

# Número máximo de pontos a serem exibidos
MAX_POINTS = 1000

# Configura matplotlib para atualização interativa
plt.ion()
fig, ax = plt.subplots()
sc = ax.scatter([], [], s=1)  # Inicializa scatter plot com pontos vazios
ax.set_xlim(-10, 10)  # Ajuste conforme a escala do seu LiDAR
ax.set_ylim(-10, 10)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("LiDAR - PointCloud2")

def callback(msg):
    # Converte os dados da nuvem de pontos
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    if len(points) == 0:
        return  # Se não houver pontos, não faz nada

    x_data, y_data = points[:, 0], points[:, 1]  # Separa X e Y

    # Atualiza o scatter plot
    sc.set_offsets(np.c_[x_data, y_data])
    ax.set_xlim(min(x_data) - 1, max(x_data) + 1)  # Ajusta dinamicamente os eixos
    ax.set_ylim(min(y_data) - 1, max(y_data) + 1)

    plt.draw()
    plt.pause(0.01)  # Pequeno delay para atualizar

def listener():
    rospy.init_node('lidar_plotter', anonymous=True)
    rospy.Subscriber('/lidar_points', PointCloud2, callback)
    rospy.spin()  # Mantém o nó rodando

if __name__ == '__main__':
    listener()
