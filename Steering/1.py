import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13

GPIO.setup(MotorA,GPIO.OUT)
GPIO.setup(MotorB,GPIO.OUT)
GPIO.setup(Enable,GPIO.OUT)

pwm = GPIO.PWM(Enable, 100)
pwm.start(0)

def control_motor(direction):
    if direction == 'right':
        GPIO.output(MotorA, GPIO.HIGH)
        GPIO.output(MotorB, GPIO.LOW)
        pwm.ChangeDutyCycle(50)
    elif direction == 'left':
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.HIGH)
        pwm.ChangeDutyCycle(50)
    else:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOW)
        pwm.ChangeDutyCycle(0)

try:
    while True:
        direction = input("Enter direction (left, right, stop): ")
        control_motor(direction)

finally:
    pwm.stop()
    GPIO.cleanup()
