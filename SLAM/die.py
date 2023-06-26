import RPi.GPIO as GPIO
import time
import socket
import json

# Motor control pins
in1 = 11
in2 = 13
en1 = 15
in3 = 16
in4 = 18
en2 = 22

# Set up GPIO pins
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

# Ultrasonic Sensor 1 GPIO pins
TRIG_PIN_1 = 20
ECHO_PIN_1 = 21

# Ultrasonic Sensor 2 GPIO pins
TRIG_PIN_2 = 4
ECHO_PIN_2 = 10

# Servo Motor 1 GPIO pin
SERVO_PIN_1 = 12

# Servo Motor 2 GPIO pin
SERVO_PIN_2 = 16

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

# TCP/IP client configuration
SERVER_IP = '192.168.1.103'  # Replace with the actual IP address of the PC
SERVER_PORT = 6000  # Replace with the desired port number

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the server
    client_socket.connect((SERVER_IP, SERVER_PORT))

    # Initialize the servo motors
    servo_pwm_1 = GPIO.PWM(SERVO_PIN_1, 50)  # 50 Hz frequency
    servo_pwm_2 = GPIO.PWM(SERVO_PIN_2, 50)  # 50 Hz frequency
    servo_pwm_1.start(0)
    servo_pwm_2.start(0)
    current_angle_1 = 0
    current_angle_2 = 0

    # Perform scanning
    direction_1 = 1  # 1 for forward, -1 for backward
    direction_2 = 1
    angle_1 = 0
    angle_2 = 0

    while True:
        if angle_1 >= 180:
            direction_1 = -1  # Change direction to move backward
        elif angle_1 <= 0:
            direction_1 = 1   # Change direction to move forward

        if angle_2 >= 180:
            direction_2 = -1  # Change direction to move backward
        elif angle_2 <= 0:
            direction_2 = 1   # Change direction to move forward

        angle_1 += direction_1 * 10  # Increment or decrement the angle by 10 degrees
        angle_2 += direction_2 * 10
        current_angle_1 = set_servo_angle(servo_pwm_1, current_angle_1, angle_1)
        current_angle_2 = set_servo_angle(servo_pwm_2, current_angle_2, angle_2)
        time.sleep(0.03)  # Delay for the servos to reach the desired angle

        dist_1 = get_distance(TRIG_PIN_1, ECHO_PIN_1)
        dist_2 = get_distance(TRIG_PIN_2, ECHO_PIN_2)
        dist_rear_left = get_distance(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT)
        dist_rear_right = get_distance(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT)
        dist_front_right = get_distance(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT)
        dist_front_left = get_distance(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT)
        
        # Create a dictionary with the sensor data
        sensor_data = {
            'distance1': dist_1,
            'distance2': dist_2,
            'rear_left': dist_rear_left,
            'rear_right': dist_rear_right,
            'front_right': dist_front_right,
            'front_left': dist_front_left,
        }
        
        # Convert the sensor data to JSON format
        data_json = json.dumps(sensor_data)

        # Send the data over the TCP/IP connection
        client_socket.sendall(data_json.encode())

        # Receive the command from the server
        command = client_socket.recv(1024).decode()

        # Process the received command
        if command == 'forward':
            forward()
            speed(100)
        elif command == 'backward':
            backward()
            speed(100)
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

        time.sleep(0.05)

except KeyboardInterrupt:
    stop()
    servo_pwm_1.stop()
    servo_pwm_2.stop()
    client_socket.close()
    GPIO.cleanup()
