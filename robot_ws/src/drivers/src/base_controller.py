#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class BaseController:
    def __init__(self):
        # Configurações do ROS
        rospy.init_node('base_controller', anonymous=True)
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        rospy.loginfo("Nó do Controlador disponível! Aguardando comandos...")

        # Configurações dos pinos GPIO (usando numeração BCM)
        GPIO.setmode(GPIO.BCM)
        self.MOTOR1_IN1 = 24
        self.MOTOR1_IN2 = 25
        self.MOTOR2_IN3 = 26
        self.MOTOR2_IN4 = 16
        self.ENABLE_A = 20
        self.ENABLE_B = 13

        # Configuração dos pinos dos encoders
        self.ENCODER1 = 5 # Exemplo, ajuste conforme necessário
        self.ENCODER2 = 6 # Exemplo, ajuste conforme necessário

        # Configuração do modo dos pinos GPIO
        GPIO.setup(self.MOTOR1_IN1, GPIO.OUT)
        GPIO.setup(self.MOTOR1_IN2, GPIO.OUT)
        GPIO.setup(self.MOTOR2_IN3, GPIO.OUT)
        GPIO.setup(self.MOTOR2_IN4, GPIO.OUT)
        GPIO.setup(self.ENABLE_A, GPIO.OUT)
        GPIO.setup(self.ENABLE_B, GPIO.OUT)

        # Configuração do modo dos encoders
        GPIO.setup(self.ENCODER1, GPIO.IN)
        GPIO.setup(self.ENCODER2, GPIO.IN)

        # Configuração do PWM para os pinos ENABLE
        self.pwm_a = GPIO.PWM(self.ENABLE_A, 1000)  # Frequência de 1000 Hz
        self.pwm_b = GPIO.PWM(self.ENABLE_B, 1000)
        self.pwm_a.start(0)  # Iniciar com duty cycle de 0%
        self.pwm_b.start(0)

        # Variáveis para contagem dos encoders
        self.encoder1_count = 0
        self.encoder2_count = 0

        # Variáveis para cálculo da velocidade angular
        self.last_encoder1_count = 0
        self.last_encoder2_count = 0
        self.last_time = rospy.Time.now().to_sec()

        # Variáveis para cálculo da velocidade angular desejada
        self.wheel_radius = 0.0375  # Raio da roda em metros
        self.wheel_separation = 0.2185  # Distância entre as rodas em metros

        # Configurações do controlador PID
        self.kp = 10.0
        self.ki = 0.0
        self.kd = 0.0
        self.last_error_a = 0
        self.integral_a = 0
        self.last_error_b = 0
        self.integral_b = 0

    def twist_callback(self, msg):
        """Callback para receber mensagens Twist."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calcular velocidades angulares desejadas
        left_wheel_vel_desired = (linear_vel - (angular_vel * self.wheel_separation / 2)) / self.wheel_radius
        right_wheel_vel_desired = (linear_vel + (angular_vel * self.wheel_separation / 2)) / self.wheel_radius

        # Calcular velocidades angulares reais
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.last_time
        encoder1_delta = self.encoder1_count - self.last_encoder1_count
        encoder2_delta = self.encoder2_count - self.last_encoder2_count
        left_wheel_vel_actual = encoder1_delta / elapsed_time
        right_wheel_vel_actual = encoder2_delta / elapsed_time

        # Calcular erros
        error_a = left_wheel_vel_desired - left_wheel_vel_actual
        error_b = right_wheel_vel_desired - right_wheel_vel_actual

        # Calcular sinais de controle PID
        self.integral_a += error_a * elapsed_time
        derivative_a = (error_a - self.last_error_a) / elapsed_time
        control_signal_a = self.kp * error_a + self.ki * self.integral_a + self.kd * derivative_a

        self.integral_b += error_b * elapsed_time
        derivative_b = (error_b - self.last_error_b) / elapsed_time
        control_signal_b = self.kp * error_b + self.ki * self.integral_b + self.kd * derivative_b

        # Enviar sinais PWM para a ponte H
        self.set_motor_speed(self.pwm_a, control_signal_a, self.MOTOR1_IN1, self.MOTOR1_IN2)
        self.set_motor_speed(self.pwm_b, control_signal_b, self.MOTOR2_IN3, self.MOTOR2_IN4)

        # Atualizar variáveis
        self.last_encoder1_count = self.encoder1_count
        self.last_encoder2_count = self.encoder2_count
        self.last_time = current_time
        self.last_error_a = error_a
        self.last_error_b = error_b

    def set_motor_speed(self, pwm, control_signal, in1_pin, in2_pin):
        """Envia sinal PWM para a ponte H."""
        speed = abs(control_signal)
        if speed > 100:
            speed = 100

        if control_signal > 0:
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        else:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)

        pwm.ChangeDutyCycle(speed)

    def run(self):
        """Executa o nó ROS."""
        rospy.spin()

    def cleanup(self):
        """Limpa os pinos GPIO ao finalizar."""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO limpo e programa finalizado.")

if __name__ == '__main__':
    try:
        controller = BaseController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()