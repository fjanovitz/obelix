#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pinos dos motores
MOTOR1_IN1 = 24
MOTOR1_IN2 = 25
MOTOR2_IN3 = 26
MOTOR2_IN4 = 16
ENABLE_A   = 20
ENABLE_B   = 13

# Configuração dos pinos
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_IN3, GPIO.OUT)
GPIO.setup(MOTOR2_IN4, GPIO.OUT)
GPIO.setup(ENABLE_A,   GPIO.OUT)
GPIO.setup(ENABLE_B,   GPIO.OUT)

# PWM nos pinos ENABLE
pwm_a = GPIO.PWM(ENABLE_A, 1000)  # Frequência de 1000 Hz
pwm_b = GPIO.PWM(ENABLE_B, 1000)

pwm_a.start(0)
pwm_b.start(0)

def mover(linear, angular):
    # Velocidade base em duty cycle (0 a 100)
    vel_linear = int(min(max(abs(linear) * 100, 0), 100))
    vel_angular = int(min(max(abs(angular) * 100, 0), 100))

    # Movimento para frente/trás
    if linear > 0:
        GPIO.output(MOTOR1_IN1, GPIO.HIGH)
        GPIO.output(MOTOR1_IN2, GPIO.LOW)
        GPIO.output(MOTOR2_IN3, GPIO.HIGH)
        GPIO.output(MOTOR2_IN4, GPIO.LOW)
    elif linear < 0:
        GPIO.output(MOTOR1_IN1, GPIO.LOW)
        GPIO.output(MOTOR1_IN2, GPIO.HIGH)
        GPIO.output(MOTOR2_IN3, GPIO.LOW)
        GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    else:
        # Movimento apenas de giro
        if angular > 0:
            GPIO.output(MOTOR1_IN1, GPIO.LOW)
            GPIO.output(MOTOR1_IN2, GPIO.HIGH)
            GPIO.output(MOTOR2_IN3, GPIO.HIGH)
            GPIO.output(MOTOR2_IN4, GPIO.LOW)
        elif angular < 0:
            GPIO.output(MOTOR1_IN1, GPIO.HIGH)
            GPIO.output(MOTOR1_IN2, GPIO.LOW)
            GPIO.output(MOTOR2_IN3, GPIO.LOW)
            GPIO.output(MOTOR2_IN4, GPIO.HIGH)
        else:
            GPIO.output(MOTOR1_IN1, GPIO.LOW)
            GPIO.output(MOTOR1_IN2, GPIO.LOW)
            GPIO.output(MOTOR2_IN3, GPIO.LOW)
            GPIO.output(MOTOR2_IN4, GPIO.LOW)

    # PWM proporcional
    pwm_a.ChangeDutyCycle(vel_linear + vel_angular)
    pwm_b.ChangeDutyCycle(vel_linear + vel_angular)

def callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z
    mover(linear, angular)

def main():
    rospy.init_node('controle_carrinho')
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
