#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, QThread, pyqtSignal

class LidarProcessor(QThread):
    """Thread separada para processar os dados do LiDAR e evitar travamento do Qt."""
    new_data = pyqtSignal(list)  # Sinal para enviar dados ao Qt

    def __init__(self):
        super().__init__()
        rospy.init_node('lidar_plotter', anonymous=True)
        self.lidar_points = []

    def run(self):
        rospy.Subscriber('/lidar_points', PointCloud2, self.lidar_callback)
        rospy.spin()  # Mantém o nó do ROS rodando

    def lidar_callback(self, msg):
        """Recebe os pontos do LiDAR e envia para a interface gráfica."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        if points:
            sampled_points = points  # Pega 1 a cada 10 pontos para reduzir carga
            xy_points = [(x, y) for x, y, _ in sampled_points]
            self.new_data.emit(xy_points)  # Envia para a GUI via sinal Qt

class LidarVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Visualização do LiDAR")
        self.setGeometry(100, 100, 600, 600)

        # Criando figura Matplotlib dentro do Qt
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.sc = self.ax.scatter([], [], s=1, c='b')

        # Layout Qt
        central_widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timer para atualizar a interface
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # Agora atualiza a cada 250ms

        # Iniciando a thread do ROS para receber os pontos
        self.lidar_thread = LidarProcessor()
        self.lidar_thread.new_data.connect(self.receive_data)
        self.lidar_thread.start()

        self.lidar_points = []

    def receive_data(self, points):
        """Recebe os pontos do LiDAR da thread e armazena para exibição."""
        self.lidar_points = points

    def update_plot(self):
        """Atualiza o gráfico com os novos pontos."""
        if self.lidar_points:
            x_data, y_data = zip(*self.lidar_points)
            self.sc.set_offsets(np.c_[x_data, y_data])
            self.canvas.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = LidarVisualizer()
    window.show()
    sys.exit(app.exec_())  # Mantém a GUI rodando
