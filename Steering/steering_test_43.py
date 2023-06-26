import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
from simple_pid import PID
import threading

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13

GPIO.setup(MotorA, GPIO.OUT)
GPIO.setup(MotorB, GPIO.OUT)
GPIO.setup(Enable, GPIO.OUT)

pwm = GPIO.PWM(Enable, 100)
pwm.start(0)

def map_range(value, left_min, left_max, right_min, right_max):
    # map the input range to the output range
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

def read_pot():
    raw_value = chan.value
    angle = map_range(raw_value, 0, 65535, 0, 270)
    return angle

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
    pid = PID(1, 0.1, 0.05)
    pid.output_limits = (-1, 1)
    while True:
        position = read_pot()  # Read potentiometer value
        if target_degree == 0:
            pid.setpoint = position  # Current position is considered as the center
        else:
            pid.setpoint = target_degree
        output = pid(position)
        control_motor(output)
        time.sleep(0.02)

motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

try:
    while True:
        new_target_degree = int(input("Enter the target degree: "))
        target_degree = new_target_degree
finally:
    pwm.stop()
    GPIO.cleanup()
