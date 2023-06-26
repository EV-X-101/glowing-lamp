import RPi.GPIO as GPIO
from simple_pid import PID
import time
import threading

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13
EncoderA = 2
EncoderB = 3

GPIO.setup(MotorA, GPIO.OUT)
GPIO.setup(MotorB, GPIO.OUT)
GPIO.setup(Enable, GPIO.OUT)
GPIO.setup(EncoderA, GPIO.IN)
GPIO.setup(EncoderB, GPIO.IN)

pwm = GPIO.PWM(Enable, 100)
pwm.start(0)

encoder_pos = 0
encoder_a_prev = 0

# Use locks for synchronization
encoder_lock = threading.Lock()
target_degree_lock = threading.Lock()

def encoder_callback(channel):
    global encoder_pos
    global encoder_a_prev
    encoder_a = GPIO.input(EncoderA)
    encoder_b = GPIO.input(EncoderB)
    if encoder_a == encoder_a_prev:
        change = (1 if encoder_b else -1)
    else:
        change = (-1 if encoder_b else 1)
    with encoder_lock:
        encoder_pos += change
    encoder_a_prev = encoder_a

GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)

def control_motor(value):
    if value > 0:
        GPIO.output(MotorA, GPIO.HIGH)
        GPIO.output(MotorB, GPIO.LOW)
    elif value < 0:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.HIGH)
    else:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOW)

    pwm.ChangeDutyCycle(abs(value) * 100)

target_degree = 0
def run_motor():
    global target_degree
    while True:
        with target_degree_lock:
            pid = PID(1, 0.1, 0.05, setpoint=target_degree)
        pid.output_limits = (-1, 1)
        while True:
            with encoder_lock:
                position = encoder_pos
            output = pid(position)
            control_motor(output)
            if abs(position - target_degree) <= 1:
                break
            time.sleep(0.02)

motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

try:
    while True:
        new_target_degree = int(input("Enter the target degree: "))
        with target_degree_lock:
            target_degree = new_target_degree
finally:
    pwm.stop()
    GPIO.cleanup()

