#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# Definir o pino GPIO para o sinal do servo
SERVO_PIN = 19

# Configurações do servo
SERVO_FREQ = 50  # Frequência do PWM (50 Hz para servos)
MIN_DUTY_CYCLE = 2.5  # Ciclo de trabalho mínimo (0 graus)
MAX_DUTY_CYCLE = 12.5  # Ciclo de trabalho máximo (180 graus)

# Configurar o modo dos pinos GPIO
GPIO.setmode(GPIO.BCM)  # Usar numeração física dos pinos
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Configurar o PWM no pino do servo
pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
pwm.start(0)  # Iniciar com duty cycle de 0%

def set_servo_angle(angle):
    """Define o ângulo do servo (0 a 180 graus)."""
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    duty_cycle = MIN_DUTY_CYCLE + (angle / 180) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)
    print(duty_cycle)
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("Testando o servomotor MG996R...")
    while True:
        # Mover o servo para 0 graus
        print("Movendo para 0 graus")
        set_servo_angle(0)
        pwm.stop()
        time.sleep(10)

        # Mover o servo para 90 graus
        print("Movendo para 90 graus")
        set_servo_angle(90)
        pwm.stop()
        time.sleep(10)

        # Mover o servo para 180 graus
        print("Movendo para 180 graus")
        set_servo_angle(180)
        pwm.stop()
        time.sleep(10)

except KeyboardInterrupt:
    pwm.ChangeDutyCycle(0)
    print("Teste interrompido pelo usuário.")

finally:
    # Parar o PWM e limpar os pinos GPIO
    pwm.stop()
    GPIO.cleanup()
    print("GPIO limpo e programa finalizado.")