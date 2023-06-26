import RPi.GPIO as GPIO
import time

# Set mode and pins
GPIO.setmode(GPIO.BOARD)

# Motor 1 pins
in1 = 11
in2 = 13
en1 = 10

# Motor 2 pins
in3 = 16
in4 = 18
en2 = 15

# Set up GPIO pins
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

# Set up PWM
p1 = GPIO.PWM(en1,100000)
p2 = GPIO.PWM(en2,100000)
p1.start(25)
p2.start(25)

# Define motor control functions
def forward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def backward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

def stop():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

def speed(duty_cycle):
    p1.ChangeDutyCycle(duty_cycle)
    p2.ChangeDutyCycle(duty_cycle)

# Main program loop
GPIO.cleanup()
while True:
    x = input("Enter a command:")
    if x == 'f':
        print("Moving forward")
        forward()
    elif x == 'b':
        print("Moving backward")
        backward()
    elif x == 's':
        print("Stopping")
        stop()
    elif x == 'l':
        print("Setting speed to low")
        speed(25)
    elif x == 'm':
        print("Setting speed to medium")
        speed(50)
    elif x == 'h':
        print("Setting speed to high")
        speed(75)
    elif x == 'mx':
        print("Running")
        speed(100)
    elif x == 'e':
        print("Exiting")
        stop()
        GPIO.cleanup()
        break
    else:
        print("Invalid command")
