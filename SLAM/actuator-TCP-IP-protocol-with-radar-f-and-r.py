import RPi.GPIO as GPIO
import time
import socket
import warnings

# Turn off GPIO warnings
warnings.filterwarnings("ignore")
GPIO.setwarnings(False)

# Set mode
GPIO.setmode(GPIO.BCM)

# Motor 1 pins
in1 = 17
in2 = 27
en1 = 22

# Motor 2 pins
in3 = 23
in4 = 24
en2 = 15

# Set up GPIO pins for motors
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)

# Set up PWM
p1 = GPIO.PWM(en1, 100000)
p2 = GPIO.PWM(en2, 100000)
p1.start(25)
p2.start(25)

# Ultrasonic sensor GPIO pins
# Rear Left Ultrasonic sensor GPIO pins
TRIG_PIN_REAR_LEFT = 11
ECHO_PIN_REAR_LEFT = 9

# Rear Right Ultrasonic sensor GPIO pins
TRIG_PIN_REAR_RIGHT = 1
ECHO_PIN_REAR_RIGHT = 0

# Front Right Ultrasonic sensor GPIO pins
TRIG_PIN_FRONT_RIGHT = 7
ECHO_PIN_FRONT_RIGHT = 25

# Front Left Ultrasonic sensor GPIO pins
TRIG_PIN_FRONT_LEFT = 26
ECHO_PIN_FRONT_LEFT = 19

# Set up GPIO pins for ultrasonic sensors
GPIO.setup(TRIG_PIN_REAR_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_REAR_LEFT, GPIO.IN)
GPIO.setup(TRIG_PIN_REAR_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_REAR_RIGHT, GPIO.IN)
GPIO.setup(TRIG_PIN_FRONT_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_FRONT_RIGHT, GPIO.IN)
GPIO.setup(TRIG_PIN_FRONT_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_FRONT_LEFT, GPIO.IN)

# Define motor control functions
def forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def speed(duty_cycle):
    p1.ChangeDutyCycle(duty_cycle)
    p2.ChangeDutyCycle(duty_cycle)

# Set up socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0', 5000))
sock.listen()
print('Socket connection established.')

# Function to measure distance using ultrasonic sensor
def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)

    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

# Main program loop
while True:
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    while True:
        data = conn.recv(1024)
        if not data:
            break
        command = data.decode('utf-8')
        print(f"Received command: {command}")
        if command == 'forward':
            forward()
            speed(100)

            # Measure distance from front ultrasonic sensors
            front_left_distance = measure_distance(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT)
            front_right_distance = measure_distance(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT)

            # Check if obstacle is detected in front
            if front_left_distance < 50 or front_right_distance < 50:
                stop()
                time.sleep(0.5)
                backward()

                # Measure distance from rear ultrasonic sensors
                rear_left_distance = measure_distance(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT)
                rear_right_distance = measure_distance(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT)

                # Check if obstacle is detected at the back
                if rear_left_distance < 50 and rear_right_distance > 50:
                    forward()
            else:
                forward()
        elif command == 'backward':
            backward()
        elif command == 'stop':
            stop()
        elif command == 'low':
            speed(25)
        elif command == 'medium':
            speed(50)
        elif command == 'high':
            speed(75)
        elif command == 'max':
            speed(100)
        elif command == 'exit':
            stop()
            GPIO.cleanup()
            break
        else:
            print("Invalid command")
    conn.close()
