import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define motor control pins
IN1 = 40  # Connect this to IN1 on L298N
IN2 = 38  # Connect this to IN2 on L298N
IN3 = 33  # Connect this to IN3 on L298N
IN4 = 31  # Connect this to IN4 on L298N

# Set up motor control pins
control_pins = [IN1, IN2, IN3, IN4]
for pin in control_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 30)

# Define the stepper motor sequence
seq = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
]

# Function to control the stepper motor
def rotate_stepper(steps, delay, direction):
    if direction == "CCW":
        steps = -steps

    for _ in range(abs(steps)):
        for step in seq[::int(steps/abs(steps))]:
            for pin, val in zip(control_pins, step):
                GPIO.output(pin, val)
            time.sleep(delay)

# Rotate the stepper motor to a specific angle
def set_angle(angle, direction="CW", step_angle=1.8, rpm=20):
    steps_per_rev = int(360 / step_angle)
    delay = 60 / (steps_per_rev * rpm)
    steps = int(angle / step_angle)

    rotate_stepper(steps, delay, direction)

if __name__ == "__main__":
    try:
        angle = float(input("Enter angle: "))
        direction = input("Enter direction: ").upper()
        rpm = float(input("Enter rpm: "))

        if direction not in ["CW", 'CCW']:
            print("Invalid direction")
        else:
            set_angle(angle, direction, rpm)

    except ValueError:
        print("Invalid input")
    finally:
        GPIO.cleanup()


