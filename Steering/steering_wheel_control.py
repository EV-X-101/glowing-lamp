import RPi.GPIO as GPIO
import time



in1 = 29
in2 = 31
en = 33
# Set up GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(in1, GPIO.OUT) # IN1
GPIO.setup(in2, GPIO.OUT) # IN2
GPIO.setup(en, GPIO.OUT) # ENA

# Set up PWM for speed control
p1 = GPIO.PWM(en, 2000) # set PWM frequency to 2 kHz
p1.start(0)

def forward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)

def backward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)

def stop():
    GPIO.output(in1,GPIO.LOW)

def speed(duty_cycle):
    p1.ChangeDutyCycle(duty_cycle)
    
while True:
    x = input("Enter a command:")
    if x == 'r':
        print("Turning Right")
        forward()
    elif x == 'l':
        print("Turning Left")
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
    elif x == 'r':
        print("Running")
        forward()
        speed(50)
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