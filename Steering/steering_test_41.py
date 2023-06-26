import RPi.GPIO as GPIO
from simple_pid import PID
import time
import threading
from spidev import SpiDev  # SPI library for ADC

GPIO.setmode(GPIO.BCM)

MotorA = 5
MotorB = 6
Enable = 13

GPIO.setup(MotorA,GPIO.OUT)
GPIO.setup(MotorB,GPIO.OUT)
GPIO.setup(Enable,GPIO.OUT)

pwm = GPIO.PWM(Enable, 100)
pwm.start(0)

spi = SpiDev()  # Create a new SPI instance
spi.open(0,0)  # Open SPI on bus 0, device 0

def read_pot(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    value = ((adc[1]&3) << 8) + adc[2]
    return value

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
encoder_center = 0
def run_motor():
    global target_degree
    global encoder_center
    pid = PID(1, 0.1, 0.05)
    pid.output_limits = (-1, 1)
    while True:
        position = read_pot(0)  # Read potentiometer value
        if target_degree == 0:
            pid.setpoint = encoder_center
        else:
            pid.setpoint = target_degree
        output = pid(position)
        control_motor(output)
        time.sleep(0.02)

# Calibration Process
input("Move motor to center position manually, then press Enter")
encoder_center = read_pot(0)  # Save the current potentiometer value as the center position
print(f"Calibrated center position: {encoder_center}")

motor_thread = threading.Thread(target=run_motor)
motor_thread.start()

try:
    while True:
        new_target_degree = int(input("Enter the target degree: "))
        target_degree = new_target_degree
finally:
    pwm.stop()
    GPIO.cleanup()
