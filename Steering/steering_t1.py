import RPi.GPIO as GPIO
from simple_pid import PID
import time

data_pin = 2
clk_pin = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(data_pin, GPIO.IN)
GPIO.setup(clk_pin, GPIO.IN)

MotorA = 5
MotorB = 6
Enable = 13  # Select a proper PWM capable GPIO pin for Enable

GPIO.setup(MotorA, GPIO.OUT)
GPIO.setup(MotorB, GPIO.OUT)
GPIO.setup(Enable, GPIO.OUT)

pid = PID(1, 0.1, 0.05, setpoint=0)  # Adjust these PID values for your specific system
pid.output_limits = (-1, 1)

pwm = GPIO.PWM(Enable, 100)  # Initialize PWM on Enable pin with 100Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle

def get_degree():
    state = (GPIO.input(data_pin), GPIO.input(clk_pin))
    if state == (0, 1):
        return 1
    elif state == (1, 0):
        return -1
    else:
        return 0

def control_motor(speed):
    # Control direction
    if speed > 0:
        GPIO.output(MotorA, GPIO.HIGH)
        GPIO.output(MotorB, GPIO.LOW)
    elif speed < 0:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.HIGH)
    else:
        GPIO.output(MotorA, GPIO.LOW)
        GPIO.output(MotorB, GPIO.LOW)

    # Control speed
    duty_cycle = abs(speed) * 100
    pwm.ChangeDutyCycle(duty_cycle)

def rotate_wheel(target_degree):
    scaling_factor = 1  # Adjust this value based on the desired motor rotation speed
    degree = 0
    start_time = time.time()

    # Reset PID controller and start rotation
    pid.reset()
    while abs(degree) < abs(target_degree):
        delta = get_degree()
        degree += delta * scaling_factor
        print("Current Degree:", degree)

        # Control the motor speed using PID
        speed = pid(degree)
        control_motor(speed)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Rotation Completed in:", elapsed_time, "seconds")

try:
    target_degree = int(input("Enter the target degree: "))
    rotate_wheel(target_degree)

finally:
    pwm.stop()  # Stop PWM
    GPIO.output(MotorA, GPIO.LOW)
    GPIO.output(MotorB, GPIO.LOW)
    GPIO.cleanup()
