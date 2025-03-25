#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
import math
from trajectory_msgs.msg import JointTrajectory
from drivers.srv import SetServo, SetServoResponse  # <-- seu novo serviço

class ControladorServo:
    def __init__(self):
        rospy.init_node('servo_controller')
        
        self.servo_pin = 19
        self.min_angle = -90
        self.max_angle = 90
        self.min_duty = 2.5
        self.max_duty = 12.5
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        self.service = rospy.Service(
            '~set_position', 
            SetServo,
            self.handle_request
        )
        
    def handle_request(self, req):
        try:
            if not req.trajectory.points:
                msg = "Requisição sem pontos recebida."
                rospy.logwarn(msg)
                return SetServoResponse(success=False, message=msg, angle=0.0)
                
            position_rad = req.trajectory.points[0].positions[0]
            angle = math.degrees(position_rad)
            duty = self.angle_to_duty(angle)
            
            self.pwm.ChangeDutyCycle(duty)
            rospy.sleep(0.3)
            
            msg = f"Servo movido para {angle:.2f}° (duty cycle: {duty:.2f}%)"
            rospy.loginfo(msg)
            return SetServoResponse(success=True, message=msg, angle=angle)
        
        except Exception as e:
            rospy.logerr(f"Erro ao processar requisição: {str(e)}")
            return SetServoResponse(success=False, message=str(e), angle=0.0)

    def angle_to_duty(self, angle):
        angle = max(self.min_angle, min(self.max_angle, angle))
        return self.min_duty + (angle - self.min_angle) * (
            (self.max_duty - self.min_duty) / (self.max_angle - self.min_angle)
        )

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    controller = ControladorServo()
    try:
        rospy.spin()
    finally:
        controller.cleanup()
