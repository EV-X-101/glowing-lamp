import RPi.GPIO as GPIO
from simple_pid import PID
import time
import threading

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13  # Select a proper PWM capable GPIO pin for Enable
EncoderA = 2
EncoderB = 3

GPIO.setup(MotorA,GPIO.OUT)
GPIO.setup(MotorB,GPIO.OUT)
GPIO.setup(Enable,GPIO.OUT)
GPIO.setup(EncoderA,GPIO.IN)
GPIO.setup(EncoderB,GPIO.IN)

encoder_pos = 0
encoder_a_prev = 0

def encoder_callback(channel):
    global encoder_pos
    global encoder_a_prev
    encoder_a = GPIO.input(EncoderA)
    encoder_b = GPIO.input(EncoderB)
    if encoder_a == encoder_a_prev:
        encoder_pos += (1 if encoder_b else -1)
    else:
        encoder_pos += (-1 if encoder_b else 1)
    encoder_a_prev = encoder_a

GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)

def control_motor(value):
    # Control direction
    if value > 0:
        GPIO.output(MotorA, GPIO.HIGH)
        GPIO.output(MotorB, GPIO.LOW)
    elif value < 0:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.HIGH)
    # else:
    #     GPIO.output(MotorA, GPIO.LOW)
    #     GPIO.output(MotorB, GPIO.LOW)

    # Control speed
    pwm.ChangeDutyCycle(abs(value) * 100)  # Change duty cycle

pwm = GPIO.PWM(Enable, 100)  # Initialize PWM on Enable pin with 100Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle

target_degree = 0  # initial target_degree
degree_set = threading.Condition()

def run_motor():
    global target_degree
    while True:
        pid = PID(1, 0.6, 0.05, setpoint=target_degree)  # Use target_degree as setpoint
        pid.output_limits = (-1, 1)
        
        while True:
            position = encoder_pos

            output = pid(position)
            control_motor(output)
            if abs(position - target_degree) <= 1:  # Check if we've reached our target
                break
            time.sleep(0.02)

# Start motor control in a new thread
motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

while True:
    target_str = input("Enter the target position (left, center, right): ")  # Get target degree from user
    if target_str == "left":
        target_degree = -90  # you can set this to your desired "left" degree
    elif target_str == "right":
        target_degree = 90  # you can set this to your desired "right" degree
    else:  # assume "center"
        target_degree = 0

    time.sleep(2)
    target_degree = 0
