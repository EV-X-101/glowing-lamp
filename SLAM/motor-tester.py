import RPi.GPIO as GPIO
import time

# GPIO pins for Motor 1
in1 = 11   # Physical pin 11
in2 = 13   # Physical pin 13
en1 = 10   # Physical pin 15

# GPIO pins for Motor 2
in3 = 16   # Physical pin 16
in4 = 18   # Physical pin 18
en2 = 15   # Physical pin 22

# Setup GPIO pins
GPIO.setmode(GPIO.BOARD)   # Set pin numbering mode to BOARD
# Set up GPIO pins
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

# Function to control Motor 1
def control_motor1():
    # Motor 1: Rotate forward
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(en1, GPIO.HIGH)
    
    # Run motor for 2 seconds
    time.sleep(2)
    
    # Stop motor
    GPIO.output(en1, GPIO.LOW)

# Function to control Motor 2
def control_motor2():
    # Motor 2: Rotate forward
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(en2, GPIO.HIGH)
    
    # Run motor for 2 seconds

    time.sleep(2)
    
    # Stop motor
    GPIO.output(en2, GPIO.LOW)

# Running both motor motors at the samae time
def control_both():
    # Motor 1: Rotate forward
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(en1, GPIO.HIGH)

    # Motor 2: Rotate forward
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(en2, GPIO.HIGH)   

# Test Motor 1
# control_motor1()

# Delay between motor tests
# time.sleep(2)

# Test Motor 2
# control_motor2()

# Cleanup GPIO pins
while True:
    try:
        control_both()
    except KeyboardInterrupt:
        GPIO.cleanup()
