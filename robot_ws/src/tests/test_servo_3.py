import RPi.GPIO as GPIO
from time import sleep

## add your servo BOARD PIN number ##
servo_pin = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm=GPIO.PWM(servo_pin, 50)
pwm.start(0)

## edit these duty cycle % values ##
left = 2.5
neutral = 7.5
right = 12
#### that's all folks ####

print("begin test")
while True:
    print("duty cycle", left,"% at left -90 deg")
    pwm.ChangeDutyCycle(left)
    sleep(5)

    print("duty cycle", neutral,"% at 0 deg")
    pwm.ChangeDutyCycle(neutral)
    sleep(5)

    print("duty cycle",right, "% at right +90 deg")
    pwm.ChangeDutyCycle(right)
    sleep(5)

print("end of test")

pwm.stop()
GPIO.cleanup()
