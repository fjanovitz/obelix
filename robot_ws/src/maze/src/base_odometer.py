#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray
import tf
import math
import time

# === CONFIGURAÇÃO DOS PINOS E PARÂMETROS ===
ENCODER_LEFT_PIN = 5
ENCODER_RIGHT_PIN = 6
PPR = 11  # Pulsos por rotação
WHEEL_DIAMETER = 2*0.0375  # metros
BASE_WIDTH = 0.2185  # Distância entre rodas, metros

DIST_PER_PULSE = math.pi * WHEEL_DIAMETER / PPR

# === VARIÁVEIS DE ODOMETRIA ===
count_left = 0
count_right = 0

x = 0.0
y = 0.0
theta = 0.0

# === CALLBACKS ===
def encoder_left_callback(channel):
    global count_left
    count_left += 1

def encoder_right_callback(channel):
    global count_right
    count_right += 1

# === NÓ PRINCIPAL ===
def main():
    global count_left, count_right, x, y, theta

    rospy.init_node('base_odometer')
    # pub = rospy.Publisher('/base_odometry', Odometry, queue_size=10)
    pub = rospy.Publisher('/base_odometry', Int32MultiArray, queue_size=10)
    br = tf.TransformBroadcaster()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_LEFT_PIN, GPIO.RISING, callback=encoder_left_callback)
    GPIO.add_event_detect(ENCODER_RIGHT_PIN, GPIO.RISING, callback=encoder_right_callback)

    rate = rospy.Rate(10)  # Hz
    last_count_left = 0
    last_count_right = 0
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        delta_left = count_left - last_count_left
        delta_right = count_right - last_count_right

        dL = delta_left * DIST_PER_PULSE
        dR = delta_right * DIST_PER_PULSE
        d = (dL + dR) / 2.0
        delta_theta = (dR - dL) / BASE_WIDTH

        # Atualizar pose
        if delta_theta != 0:
            x += d * math.cos(theta + delta_theta / 2.0)
            y += d * math.sin(theta + delta_theta / 2.0)
            theta += delta_theta
        else:
            x += d * math.cos(theta)
            y += d * math.sin(theta)

        # Criar mensagem Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))

        # Velocidades (linear e angular)
        odom.twist.twist.linear.x = d / dt
        odom.twist.twist.angular.z = delta_theta / dt

        # Publicar odometria e tf
        msg = Int32MultiArray()
        msg.data = [delta_left, delta_right]
        pub.publish(msg)
        # pub.publish(odom)
        # br.sendTransform((x, y, 0.0),
        #                  tf.transformations.quaternion_from_euler(0, 0, theta),
        #                  now,
        #                  "base_link",
        #                  "odom")

        # Atualizar variáveis
        last_count_left = count_left
        last_count_right = count_right
        last_time = now
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    main()
