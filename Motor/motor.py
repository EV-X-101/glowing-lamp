import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)


pwm1 = GPIO.PWM(23, 100)
pwm2 = GPIO.PWM(25, 100)
pwm1.start(0)
pwm2.start(0)



def motor_control(speed, angle, direction):
    
    if direction == "f":
        print("Forward")
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(23, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        print("Forward End")
    elif direction == "b":
        print("backward")
        GPIO.output(22, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(25, GPIO.HIGH)
        print("back end")
    else:
        GPIO.output(22, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        print("stop")
    
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    
    
    angle_steps = int(angle * 16/ 360)
    for i in range(angle_steps):
        GPIO.output(22, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(22, GPIO.LOW)
        time.sleep(0.001)
    


motor_control(50, 90, "f")
time.sleep(2)

motor_control(25, 180, "b")
time.sleep(2)

motor_control(75, 270, "f")
time.sleep(2)


motor_control(0,0, "s")


GPIO.cleanup()
        