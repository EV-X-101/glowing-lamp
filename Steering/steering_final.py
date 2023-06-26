import RPi.GPIO as GPIO
import time
from simple_pid import PID
import serial
import time
import re
from ads1115_steering_angle_measurement import measured_angle

# Set up RPi.GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the GPIO pins for the L298N
in1_pin = 29
in2_pin = 31

GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

# Function to set the motor direction
def set_motor_direction(in1, in2):
    GPIO.output(in1_pin, in1)
    GPIO.output(in2_pin, in2)

# Function to stop the motor
def stop_motor():
    set_motor_direction(GPIO.LOW, GPIO.LOW)

# Read the angle from the potentiometer

# Set up the PID controller
pid = PID(1, 1, 0.01)
pid.output_limits = (-1, 1)  # the motor can go in both directions

# Main loop
try:
    while True:

        # Get the desired angle from the user
        desired_angle = int(input("Enter the desired angle (0-360): "))
        pid.setpoint = desired_angle

        # Read the angle
        angle = int(measured_angle())
        print(angle)

        # Compute the PID controller output
        control = pid(angle)
        print(control)

        if control > 0:
            # Spin the motor in one direction
            print(f'Hello, Leul')
            set_motor_direction(GPIO.HIGH, GPIO.LOW)
        elif control < 0:
            # Spin the motor in the other direction
            print(f'Hello, Mee')
            set_motor_direction(GPIO.LOW, GPIO.HIGH)
        else:
            # Stop the motor
            print(f'Hello, Dear')
            stop_motor()

        # Wait a bit
        time.sleep(0.1)
except KeyboardInterrupt:
    # Clean up GPIO on CTRL+C exit
    stop_motor()
    GPIO.cleanup()
