import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

# Definir o pino GPIO para o sinal PWM do servo
SERVO_PIN = 19

# Parâmetros do servo MG996R
SERVO_FREQ = 50  # Frequência do PWM (50 Hz)
MIN_DUTY_CYCLE = 2.5  # Para ângulo 0°
MAX_DUTY_CYCLE = 12.5  # Para ângulo 180°

# Configuração do GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Criar PWM no pino do servo
pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
pwm.start(7.5)  # Inicializa em 90°

def set_servo_angle(angle):
    """Define o ângulo do servo (0 a 180 graus)."""
    angle = max(0, min(180, angle))  # Garante que o ângulo esteja no intervalo válido
    duty_cycle = MIN_DUTY_CYCLE + (angle / 180) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)
    
    rospy.loginfo(f"Movendo servo para {angle} graus (Duty Cycle: {duty_cycle:.2f}%)")
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)  # Aguarda o movimento do servo

def servo_callback(msg):
    """Callback ROS para mover o servo baseado no tópico /servo_angle."""
    set_servo_angle(msg.data)

def servo_node():
    """Inicializa o nó ROS para controlar o servomotor."""
    rospy.init_node("servo_controller", anonymous=True)
    rospy.Subscriber("/servo_angle", Float32, servo_callback)
    rospy.loginfo("Nó do servo iniciado. Aguarde comandos em /servo_angle.")
    
    rospy.spin()  # Mantém o nó rodando

    # Limpeza dos pinos GPIO ao finalizar
    pwm.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        servo_node()
    except rospy.ROSInterruptException:
        pass
