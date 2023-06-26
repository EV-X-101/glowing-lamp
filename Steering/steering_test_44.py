import RPi.GPIO as GPIO
from simple_pid import PID
import time
import threading
from smbus2 import SMBus, i2c_msg

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13

GPIO.setup(MotorA,GPIO.OUT)
GPIO.setup(MotorB,GPIO.OUT)
GPIO.setup(Enable,GPIO.OUT)

pwm = GPIO.PWM(Enable, 100)
pwm.start(0)

I2C_ADDRESS = 0x48  # Default I2C address for ADS1115, change if necessary

def read_adc(channel):
    with SMBus(1) as bus:  # 1 indicates /dev/i2c-1
        msg = i2c_msg.write(I2C_ADDRESS, [0x01, 0x83, 0x83])
        bus.i2c_rdwr(msg)
        time.sleep(0.05)
        msg = i2c_msg.write(I2C_ADDRESS, [0x00])
        bus.i2c_rdwr(msg)
        msg = i2c_msg.read(I2C_ADDRESS, 2)
        bus.i2c_rdwr(msg)
        result = msg.buf[0][0] << 8 | msg.buf[1][0]
        if result >= 0x8000:
            return result - 65536
        else:
            return result

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

def map_range(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

target_degree = 0
encoder_center = 0
def run_motor():
    global target_degree
    global encoder_center
    pid = PID(4, 0.4, 0.05)
    pid.output_limits = (-1, 1)
    pid.setpoint = encoder_center
    while True:
        position = read_adc(0)
        if target_degree != 0:
            target_adc = map_range(target_degree, 0, 270, 0, 65535)
            pid.setpoint = target_adc
        output = pid(position)
        control_motor(output)
        time.sleep(0.02)

# Calibration Process
input("Move motor to center position manually, then press Enter")
encoder_center = read_adc(0)  # Save the current encoder value as the center position
print(f"Calibrated center position is {encoder_center}")

motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

try:
    while True:
        new_target_degree = int(input("Enter the target degree: "))
        target_degree = new_target_degree
finally:
    pwm.stop()
    GPIO.cleanup()
