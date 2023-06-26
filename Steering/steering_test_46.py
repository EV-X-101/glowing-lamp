import RPi.GPIO as GPIO
from simple_pid import PID
from smbus2 import SMBus, i2c_msg
import time
import threading

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13

GPIO.setup(MotorA, GPIO.OUT)
GPIO.setup(MotorB, GPIO.OUT)
GPIO.setup(Enable, GPIO.OUT)

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

def map_range(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

def control_motor(value):
    if value > 0:
        GPIO.output(MotorA, GPIO.HIGH)
        GPIO.output(MotorB, GPIO.LOW)
    elif value < 0:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.HIGH)
    else:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOw)
    pwm.ChangeDutyCycle(abs(value) * 100)

target_adc = 0
min_adc = 0
max_adc = 65535

def run_motor():
    global target_adc, min_adc, max_adc
    Kp = 4
    Ki = 0.4
    Kd = 0.05
    pid = PID(Kp, Ki, Kd)
    pid.output_limits = (-1, 1)
    while True:
        adc = read_adc(0)
        if min_adc <= adc <= max_adc:
            pid.setpoint = target_adc
            output = pid(adc)
            control_motor(output)
            print(f"ADC: {adc}, Target: {target_adc}, PID Output: {output}")
        else:
            control_motor(0)
        time.sleep(0.02)

try:
    print("Adjust the steering to the right position and press Enter")
    input()
    min_adc = read_adc(0)
    print("Right boundary set to ADC: ", min_adc)

    print("Adjust the steering to the center position and press Enter")
    input()
    center_adc = read_adc(0)
    print("Center position set to ADC: ", center_adc)

    print("Adjust the steering to the left position and press Enter")
    input()
    max_adc = read_adc(0)
    print("Left boundary set to ADC: ", max_adc)

    motor_thread = threading.Thread(target=run_motor)
    motor_thread.start()

    while True:
        target_angle = int(input("Enter the target angle between -135 and 135 degrees: "))
        target_adc = map_range(target_angle, -135, 135, min_adc, max_adc)
        print("Target ADC value: ", target_adc)
finally:
    pwm.stop()
    GPIO.cleanup()
