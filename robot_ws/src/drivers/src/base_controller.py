#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist

# ------------------------------------------------------
# CONFIGURAÇÃO DE GPIO
# ------------------------------------------------------
GPIO.setmode(GPIO.BCM)  # Usamos a numeração BCM

# -------------------------
# PINOS MOTORES (já existentes)
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

# PWM nos pinos ENABLE
pwm_a = GPIO.PWM(ENABLE_A, 1000)  # Frequência de 1000 Hz
pwm_b = GPIO.PWM(ENABLE_B, 1000)
pwm_a.start(0)
pwm_b.start(0)

# -------------------------
# PINOS CODIFICADORES
# -------------------------
# (Exemplo de uso de 1 canal por motor)
ENCODER_LEFT = 5  # Exemplo: GPIO 5
ENCODER_RIGHT = 6 # Exemplo: GPIO 6

# Se o codificador tivesse segundo canal, comentaríamos, por ex.:
# ENCODER_LEFT_B = 12  # ok (não usado neste exemplo)
# ENCODER_RIGHT_B = 17 # ok (não usado neste exemplo)

GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# ------------------------------------------------------
# VARIÁVEIS GLOBAIS PARA A LÓGICA
# ------------------------------------------------------
# Contador de pulsos para cada motor
left_count = 0
right_count = 0

# Taxa de atualização (Hz) - definiremos quando iniciar o ROS node
UPDATE_RATE = 20.0  # por exemplo, 20 Hz

# Constantes do robô
R         = 0.03  # Raio da roda (ex: 3 cm)
PPR       = 11    # Pulsos por volta do encoder (exemplo)
gear_ratio= 1.0   # Se houver caixa de redução adicional, colocar aqui (46, 50 etc.)
# Se não souber, deixe 1.0 e comente ao lado: "exemplo"

# Constantes do controlador (exemplo de PID simplificado)
KP = 30.0
KI = 0.0
KD = 0.0

# Erro acumulado e último erro para o termo integrativo e derivativo
erro_acumulado_esq  = 0.0
erro_acumulado_dir  = 0.0
erro_anterior_esq   = 0.0
erro_anterior_dir   = 0.0

# Velocidades desejadas (vindas do Twist) e medidas
vel_desejada_linear = 0.0
vel_desejada_angular= 0.0

# ------------------------------------------------------
# FUNÇÕES DE AJUSTE DE PWM (ENVIO PARA A PONTE H)
# ------------------------------------------------------
def motor1_forward(speed):
    """ Motor esquerdo para frente """
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)

def motor1_backward(speed):
    """ Motor esquerdo para trás """
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)

def motor2_forward(speed):
    """ Motor direito para frente """
    GPIO.output(MOTOR2_IN3, GPIO.HIGH)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(speed)

def motor2_backward(speed):
    """ Motor direito para trás """
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(speed)

def stop_motors():
    """ Para ambos os motores """
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

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
    global vel_desejada_linear, vel_desejada_angular
    vel_desejada_linear  = msg.linear.x
    vel_desejada_angular = msg.angular.z

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
    global erro_acumulado_esq, erro_acumulado_dir
    global erro_anterior_esq,  erro_anterior_dir

    rate = rospy.Rate(UPDATE_RATE)

    # Parâmetro: tempo entre cada iteração
    Ts = 1.0 / UPDATE_RATE

    while not rospy.is_shutdown():
        # 1) Ler contagem do encoder e zerar
        pulsos_esq = left_count
        pulsos_dir = right_count
        left_count  = 0
        right_count = 0

        # 2) Calcular velocidade angular real (rad/s)
        # pulsos -> voltas -> rad/s
        # pulsos_esq / PPR = número de voltas no intervalo
        # Como gear_ratio é a razão de redução (ex. 46),
        # a roda gira (1/gear_ratio) voltas para cada volta do eixo motor, se for esse o caso.
        vel_angular_esq = (2.0 * 3.1415) * (pulsos_esq / (PPR*gear_ratio)) / Ts
        vel_angular_dir = (2.0 * 3.1415) * (pulsos_dir / (PPR*gear_ratio)) / Ts

        # 3) Calcular velocidade angular desejada de cada roda, a partir de (v, w)
        #   v = (wr_esq + wr_dir)/2
        #   w = (wr_dir - wr_esq)/L (distância entre rodas)
        #   => wr_esq = v/R - (w*L)/(2*R)
        #   => wr_dir = v/R + (w*L)/(2*R)
        # Supondo L = distancia_entre_rodas = 0.14  (exemplo)
        L = 0.20
        w_esq_desej = (vel_desejada_linear / R) - (vel_desejada_angular * (L/2.0) / R)
        w_dir_desej = (vel_desejada_linear / R) + (vel_desejada_angular * (L/2.0) / R)

        # 4) Controlador PID (simplificado). A saída do PID será o Duty Cycle [0..100]
        # erro = w_desejada - w_medida
        erro_esq = w_esq_desej - vel_angular_esq
        erro_dir = w_dir_desej - vel_angular_dir

        # Acumular erro (integral)
        erro_acumulado_esq += erro_esq * Ts
        erro_acumulado_dir += erro_dir * Ts

        # Derivada do erro
        deriv_esq = (erro_esq - erro_anterior_esq) / Ts
        deriv_dir = (erro_dir - erro_anterior_dir) / Ts

        # PID
        controle_esq = KP*erro_esq + KI*erro_acumulado_esq + KD*deriv_esq
        controle_dir = KP*erro_dir + KI*erro_acumulado_dir + KD*deriv_dir

        # Atualiza "erro_anterior"
        erro_anterior_esq = erro_esq
        erro_anterior_dir = erro_dir

        # 5) Enviar sinal PWM (com saturação entre 0% e 100%)
        pwm_esq = controle_esq
        pwm_dir = controle_dir

        # Saturação
        if pwm_esq > 100: pwm_esq = 100
        if pwm_esq < -100: pwm_esq = -100
        if pwm_dir > 100: pwm_dir = 100
        if pwm_dir < -100: pwm_dir = -100

        # Se pwm for negativo, giramos para trás
        if pwm_esq >= 0:
            motor1_forward(pwm_esq)
        else:
            motor1_backward(abs(pwm_esq))

        if pwm_dir >= 0:
            motor2_forward(pwm_dir)
        else:
            motor2_backward(abs(pwm_dir))

        rate.sleep()

# ------------------------------------------------------
# MAIN
# ------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("controlador_base_node", anonymous=False)

        # Registrar callbacks de interrupção do encoder
        GPIO.add_event_detect(ENCODER_LEFT,  GPIO.RISING, callback=callback_encoder_left)
        GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=callback_encoder_right)

        # Subscriber para velocidade desejada
        rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)

        rospy.loginfo("ControladorBase iniciado! Aguardando comandos Twist em /cmd_vel ...")
        controlador_base()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass

    finally:
        stop_motors()
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO limpo e programa finalizado.")
