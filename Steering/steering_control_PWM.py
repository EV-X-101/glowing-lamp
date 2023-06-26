import RPi.GPIO as GPIO
import time
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board
import busio

# Pin definitions
MotorIn1 = 29     # Connect to IN1 on L298N
MotorIn2 = 31     # Connect to IN2 on L298N
MotorEnable = 33  # Connect to ENA on L298N

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MotorIn1, GPIO.OUT)
GPIO.setup(MotorIn2, GPIO.OUT)
GPIO.setup(MotorEnable, GPIO.OUT)

# Set the motor direction
GPIO.output(MotorIn1, GPIO.HIGH)
GPIO.output(MotorIn2, GPIO.LOW)

# Start PWM at 0 duty cycle (off)
motor = GPIO.PWM(MotorEnable, 100)  # Initialize PWM on pwmPin 100Hz frequency
motor.start(0)

def read_current_angle():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    chan = AnalogIn(ads, ADS.P0)

    # Convert voltage to angle
    angle = (chan.voltage / 3.3) * 180.0  # Assuming 0V = 0 degrees and 3.3V = 180 degrees

    return angle

try:
    while True:
        desired_angle = float(input("Enter the desired angle (0-180): "))
        desired_duty_cycle = abs(desired_angle - 90) / 90.0 * 100  # Convert angle deviation from 90 to duty cycle
        print(desired_duty_cycle)
        current_angle = read_current_angle()
        print(current_angle)
        while abs(current_angle - desired_angle) > 1:  # Allow error of 1 degree
            if current_angle < desired_angle:  # Rotate right
                GPIO.output(MotorIn1, GPIO.HIGH)
                GPIO.output(MotorIn2, GPIO.LOW)
            else:  # Rotate left
                GPIO.output(MotorIn1, GPIO.LOW)
                GPIO.output(MotorIn2, GPIO.HIGH)

            motor.ChangeDutyCycle(desired_duty_cycle)
            current_angle = read_current_angle()
            time.sleep(0.01)  # Wait for 0.01 second

        motor.ChangeDutyCycle(0)  # Stop the motor

except KeyboardInterrupt:
    print("User interrupted, stopping.")

# Stop PWM and cleanup GPIOs
motor.stop()
GPIO.cleanup()
