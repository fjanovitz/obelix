#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
import math
from trajectory_msgs.msg import JointTrajectory
from drivers.srv import Servo, ServoResponse

class ControladorServo:
    def __init__(self):
        rospy.init_node('servo_controller')
        
        self.servo_pin = 19
        self.min_angle = -90
        self.max_angle = 90
        self.min_duty = 2.5
        self.max_duty = 12.5
        self.current_angle = 0.0
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        self.service = rospy.Service('~set_position', Servo, self.handle_request)

        rospy.loginfo("Serviço de controle do servo iniciado. Aguardando requisições...")	
        
    def handle_request(self, req):
        try:
            if not req.trajectory.points:
                msg = "Requisição sem pontos recebida."
                rospy.logwarn(msg)
                return ServoResponse(success=False, message=msg, angle=self.current_angle)
                
            position_rad = req.trajectory.points[0].positions[0]
            angle = math.degrees(position_rad)
            
            if angle < self.min_angle or angle > self.max_angle:
                msg = f"Ângulo {angle:.2f}° fora dos limites [{self.min_angle}, {self.max_angle}]"
                rospy.logwarn(msg)
                return ServoResponse(success=False, message=msg, angle=self.current_angle)
            
            if angle == self.current_angle:
                msg = f"Servo já está na posição {angle:.2f}°"
                rospy.loginfo(msg)
                return ServoResponse(success=True, message=msg, angle=self.current_angle)
            
            duty = self.angle_to_duty(angle)
            
            duty = max(self.min_duty, min(self.max_duty, duty))
            
            rospy.loginfo(f"Enviando duty cycle: {duty:.2f}% para o ângulo {angle:.2f}°")
            self.pwm.ChangeDutyCycle(duty)
            rospy.sleep(0.3)
            self.pwm.ChangeDutyCycle(0)
            
            self.current_angle = angle
            
            msg = f"Servo movido para {angle:.2f}° (duty cycle: {duty:.2f}%)"
            rospy.loginfo(msg)
            return ServoResponse(success=True, message=msg, angle=angle)
        
        except Exception as e:
            rospy.logerr(f"Erro ao processar requisição: {str(e)}")
            return ServoResponse(success=False, message=str(e), angle=self.current_angle)

    def angle_to_duty(self, angle):
        normalized_angle = max(self.min_angle, min(self.max_angle, angle))
        return ((normalized_angle - self.min_angle) * (self.max_duty - self.min_duty) / 
                (self.max_angle - self.min_angle)) + self.min_duty

    def cleanup(self):
        self.pwm.ChangeDutyCycle(0)
        self.pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    controller = ControladorServo()
    try:
        rospy.spin()
    finally:
        controller.cleanup()