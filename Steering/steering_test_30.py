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

pwm = GPIO.PWM(Enable, 100)  # Initialize PWM on Enable pin with 100Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle

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
    else:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOW)

    # Control speed
    pwm.ChangeDutyCycle(abs(value) * 100)  # Change duty cycle

target_degree = 0
degree_set = threading.Condition()

def run_motor():
    global target_degree
    global encoder_pos
    pid = PID(0.4, 0.1, 0.05, setpoint=0)  # Adjust these PID values for your specific system
    pid.output_limits = (-1, 1)
    
    try:
        while True:
            with degree_set:
                degree_set.wait()  # Wait for a new target degree
            pid.setpoint = target_degree
            while abs(encoder_pos - target_degree) > 1:  # Run until target degree is achieved
                output = pid(encoder_pos)
                control_motor(output)
                time.sleep(0.02)
    finally:
        pwm.stop()  # stop PWM
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOW)
        GPIO.cleanup()

motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

# Start with manual calibration
print("Manually set the steering to center and press enter...")
input()
encoder_pos = 0  # Set the current position as the center
print("Calibration done. The current position is set as center.")

while True:
    target_degree = int(input("Enter the target degree: "))  # Get target degree from user
    with degree_set:
        degree_set.notify()  # Signal the motor control thread
