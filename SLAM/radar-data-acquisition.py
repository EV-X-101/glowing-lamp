import RPi.GPIO as GPIO
import time
import socket
import json

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

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN_1, GPIO.OUT)
GPIO.setup(ECHO_PIN_1, GPIO.IN)
GPIO.setup(TRIG_PIN_2, GPIO.OUT)
GPIO.setup(ECHO_PIN_2, GPIO.IN)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)
GPIO.setup(SERVO_PIN_2, GPIO.OUT)
GPIO.setup(TRIG_PIN_REAR_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_REAR_LEFT, GPIO.IN)
GPIO.setup(TRIG_PIN_REAR_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_REAR_RIGHT, GPIO.IN)
GPIO.setup(TRIG_PIN_FRONT_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_FRONT_RIGHT, GPIO.IN)
GPIO.setup(TRIG_PIN_FRONT_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_FRONT_LEFT, GPIO.IN)

# Create a PWM object for the servo motors
servo_pwm_1 = GPIO.PWM(SERVO_PIN_1, 50)  # 50 Hz frequency
servo_pwm_2 = GPIO.PWM(SERVO_PIN_2, 50)  # 50 Hz frequency

# Function to set the angle of the servo motor
def set_servo_angle(servo_pwm, current_angle, target_angle):
    if target_angle < 0:
        target_angle = 0
    elif target_angle > 180:
        target_angle = 180
    
    if current_angle < 0:
        current_angle = 0
    elif current_angle > 180:
        current_angle = 180

    angle_diff = abs(target_angle - current_angle)
    increment = 1 if target_angle > current_angle else -1

    for angle in range(current_angle, target_angle, increment):
        duty_cycle = angle / 18.0 + 2.5
        servo_pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.01)  # Adjust the delay as per your requirement
    
    # Update the current angle to the target angle
    current_angle = target_angle
    
    return current_angle

# Function to get distance from ultrasonic sensor
def get_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_start = 0
    pulse_end = 0

    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the server
    client_socket.connect((SERVER_IP, SERVER_PORT))

    # Initialize the servo motors
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

        time.sleep(0.05)

except KeyboardInterrupt:
    servo_pwm_1.stop()
    servo_pwm_2.stop()
    client_socket.close()
    GPIO.cleanup()
