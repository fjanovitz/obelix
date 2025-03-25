#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# Definir os pinos GPIO (usando numeração física)
GPIO.setmode(GPIO.BCM)  # Usar numeração BCM
MOTOR1_IN1 = 24  
MOTOR1_IN2 = 25  
MOTOR2_IN3 = 26  
MOTOR2_IN4 = 16  
ENABLE_A = 20         
ENABLE_B = 13

# Configurar o modo dos pinos GPIO 
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_IN3, GPIO.OUT)
GPIO.setup(MOTOR2_IN4, GPIO.OUT)
GPIO.setup(ENABLE_A, GPIO.OUT)
GPIO.setup(ENABLE_B, GPIO.OUT)

# Configurar PWM para os pinos ENABLE
pwm_a = GPIO.PWM(ENABLE_A, 1000)  # Frequência de 1000 Hz
pwm_b = GPIO.PWM(ENABLE_B, 1000)
pwm_a.start(0)  # Iniciar com duty cycle de 0%
pwm_b.start(0)

def motor1_forward(speed):
    """Move o Motor 1 para frente com uma velocidade específica."""
    GPIO.output(MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)

def motor1_backward(speed):
    """Move o Motor 1 para trás com uma velocidade específica."""
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)

def motor2_forward(speed):
    """Move o Motor 2 para frente com uma velocidade específica."""
    GPIO.output(MOTOR2_IN3, GPIO.HIGH)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(speed)

def motor2_backward(speed):
    """Move o Motor 2 para trás com uma velocidade específica."""
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(speed)

def stop_motors():
    """Para ambos os motores."""
    GPIO.output(MOTOR1_IN1, GPIO.LOW)
    GPIO.output(MOTOR1_IN2, GPIO.LOW)
    GPIO.output(MOTOR2_IN3, GPIO.LOW)
    GPIO.output(MOTOR2_IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

try:
    # Testar o Motor 1
    print("Motor 1 para frente (100% de velocidade)")
    motor1_forward(100)
    time.sleep(2)  # Rodar por 2 segundos
    stop_motors()

    print("Motor 1 para trás (100% de velocidade)")
    motor1_backward(100)
    time.sleep(2)
    stop_motors()

    # Testar o Motor 2
    print("Motor 2 para frente (100% de velocidade)")
    motor2_forward(100)
    time.sleep(2)
    stop_motors()

    print("Motor 2 para trás (100% de velocidade)")
    motor2_backward(100)
    time.sleep(2)
    stop_motors()

    # Testar ambos os motores juntos
    print("Ambos os motores para frente (100% de velocidade)")
    motor1_forward(100)
    motor2_forward(100)
    time.sleep(2)
    stop_motors()
    
	# Testar ambos os motores juntos
    print("Ambos os motores para frente (100% de velocidade)")
    motor1_backward(100)
    motor2_backward(100)
    time.sleep(2)
    stop_motors()

except KeyboardInterrupt:
    print("Teste interrompido pelo usuário.")

finally:
    # Limpar os pinos GPIO ao finalizar
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("GPIO limpo e programa finalizado.")