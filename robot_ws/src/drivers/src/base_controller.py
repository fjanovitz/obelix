#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import RPi.GPIO as GPIO
import time
import math
from geometry_msgs.msg import Twist

# ------------------------------------------------------
# CONFIGURAÇÃO DE GPIO
# ------------------------------------------------------
GPIO.setmode(GPIO.BCM)  # Usamos a numeração BCM

# -------------------------
# PINOS MOTORES E ENABLERS
# -------------------------
MOTOR1_IN1 = 24
MOTOR1_IN2 = 25
MOTOR2_IN3 = 26
MOTOR2_IN4 = 16
ENABLE_A   = 20
ENABLE_B   = 13

GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_IN3, GPIO.OUT)
GPIO.setup(MOTOR2_IN4, GPIO.OUT)
GPIO.setup(ENABLE_A,   GPIO.OUT)
GPIO.setup(ENABLE_B,   GPIO.OUT)

pwm_a = GPIO.PWM(ENABLE_A, 1000)  # Frequência de 1000 Hz
pwm_b = GPIO.PWM(ENABLE_B, 1000)
pwm_a.start(0)
pwm_b.start(0)

# -------------------------
# PINOS CODIFICADORES
# -------------------------

ENCODER_LEFT = 5  # Exemplo: GPIO 5
ENCODER_RIGHT = 6 # Exemplo: GPIO 6

GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# ------------------------------------------------------
# VARIÁVEIS GLOBAIS PARA A LÓGICA
# ------------------------------------------------------
# Contador de pulsos para cada motor
left_count = 0
right_count = 0

# Taxa de atualização (Hz)
UPDATE_RATE = 20.0

# Constantes do robô
D          = 0.2185
R          = 0.0375 
PPR        = 11
gear_ratio = 1.0

# Constantes do controlador PID
KP = 0.0
KI = 0.0
KD = 0.0

# Erro acumulado e último erro para o termo integrativo e derivativo
acc_error_left   = 0.0
acc_error_right  = 0.0
prev_error_left  = 0.0
prev_error_right = 0.0

# Velocidades desejadas (vindas do Twist) e medidas
linear_vel  = 0.0
angular_vel = 0.0

# ------------------------------------------------------
# FUNÇÕES DE AJUSTE DE PWM (ENVIO PARA A PONTE H)
# ------------------------------------------------------

def set_left_speed(speed):
    """ Define a velocidade do motor esquerdo (0 a 100) """
    if speed > 0:
        GPIO.output(MOTOR1_IN1, GPIO.HIGH)
        GPIO.output(MOTOR1_IN2, GPIO.LOW)
    elif speed == 0:
        GPIO.output(MOTOR1_IN1, GPIO.LOW)
        GPIO.output(MOTOR1_IN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR1_IN1, GPIO.LOW)
        GPIO.output(MOTOR1_IN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)

def set_right_speed(speed):
    """ Define a velocidade do motor direito (0 a 100) """
    if speed > 0:
        GPIO.output(MOTOR2_IN3, GPIO.HIGH)
        GPIO.output(MOTOR2_IN4, GPIO.LOW)
    elif speed == 0:
        GPIO.output(MOTOR2_IN3, GPIO.LOW)
        GPIO.output(MOTOR2_IN4, GPIO.LOW)
    else:
        GPIO.output(MOTOR2_IN3, GPIO.LOW)
        GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(speed)

# ------------------------------------------------------
# CALLBACK DOS ENCODERS
# Usamos interrupção para contar pulsos
# ------------------------------------------------------
def callback_encoder_left(channel):
    global left_count
    left_count += 1

def callback_encoder_right(channel):
    global right_count
    right_count += 1

# ------------------------------------------------------
# CALLBACK DO TWIST (velocidade linear e angular desejadas)
# ------------------------------------------------------
def callback_cmd_vel(msg):
    """
    Recebe a velocidade linear (m/s) e angular (rad/s).
    Armazena para ser usada no loop de controle.
    """
    global linear_vel, angular_vel
    linear_vel  = msg.linear.x
    angular_vel = msg.angular.z

# ------------------------------------------------------
# FUNÇÃO PRINCIPAL DE CONTROLE
# ------------------------------------------------------
def controlador_base():
    """
    - Conta pulsos em certo intervalo
    - Calcula velocidade real
    - Calcula velocidade desejada de cada roda
    - Calcula PID
    - Aplica PWM
    """

    global left_count, right_count
    global acc_error_left, acc_error_right
    global prev_error_left,  prev_error_right

    rate = rospy.Rate(UPDATE_RATE)

    # Parâmetro: tempo entre cada iteração
    Ts = 1.0 / UPDATE_RATE

    while not rospy.is_shutdown():
        # 1) Ler contagem do encoder e zerar
        pulse_counter_left = left_count
        pulse_counter_right = right_count
        left_count  = 0
        right_count = 0

        # 2) Calcular velocidade angular real (rad/s)
        # pulsos -> voltas -> rad/s
        # pulse_counter_left / PPR = número de voltas no intervalo
        # Como gear_ratio é a razão de redução (ex. 46),
        # a roda gira (1/gear_ratio) voltas para cada volta do eixo motor, se for esse o caso.
        angular_vel_left = (2.0 * math.pi) * (pulse_counter_left / (PPR*gear_ratio)) / Ts
        angular_vel_right = (2.0 * math.pi) * (pulse_counter_right / (PPR*gear_ratio)) / Ts

        # 3) Calcular velocidade angular desejada de cada roda, a partir de (v, w)
        # Equações inversas:
        # uL = v/R - D*w/R
        # uR = v/R + D*w/R
        des_angular_left = (linear_vel / R) - (angular_vel * D / R)
        des_angular_right = (linear_vel / R) + (angular_vel * D / R)

        # 4) Controlador PID (simplificado). A saída do PID será o Duty Cycle [0..100]
        # erro = w_desejada - w_medida
        error_left = des_angular_left - angular_vel_left
        error_right = des_angular_right - angular_vel_right

        # Acumular erro (integral)
        acc_error_left += error_left * Ts
        acc_error_right += error_right * Ts

        # Derivada do erro
        deriv_left = (error_left - prev_error_left) / Ts
        deriv_right = (error_right - prev_error_right) / Ts

        # PID
        control_left = KP*error_left + KI*acc_error_left + KD*deriv_left
        control_right = KP*error_right + KI*acc_error_right + KD*deriv_right

        # Atualiza "prev_error"
        prev_error_left = error_left
        prev_error_right = error_right

        # 5) Enviar sinal PWM (com saturação entre 0% e 100%)
        pwm_left = control_left
        pwm_right = control_right

        # Saturação
        if pwm_left > 100: pwm_left = 100
        if pwm_left < -100: pwm_left = -100
        if pwm_right > 100: pwm_right = 100
        if pwm_right < -100: pwm_right = -100

        # Se pwm for negativo, giramos para trás
        set_left_speed(pwm_left)
        set_right_speed(pwm_right)

        rate.sleep()

# ------------------------------------------------------
# MAIN
# ------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("base_controller", anonymous=False)

        # Registrar callbacks de interrupção do encoder
        GPIO.add_event_detect(ENCODER_LEFT,  GPIO.RISING, callback=callback_encoder_left)
        GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=callback_encoder_right)

        # Subscriber para velocidade desejada
        rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)

        rospy.loginfo("Nó do Controlador iniciado! Aguardando comandos no formato Twist em /cmd_vel ...")
        controlador_base()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass

    finally:
        set_left_speed(0)
        set_right_speed(0)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO limpo e programa finalizado.")
