import RPi.GPIO as GPIO
import time
import sys

# Pin Definitions
motor_pin1 = 29
motor_pin2 = 31
enable_pin = 33
encoder_pin_a = 35
encoder_pin_b = 37

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(encoder_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up PWM for motor speed control
motor_pwm = GPIO.PWM(enable_pin, 1000)  # Set PWM frequency to 1000 Hz
motor_pwm.start(0)

# Define motor control functions
def set_motor_speed(speed):
    duty_cycle = max(0, min(100, abs(speed)))  # Limit duty cycle to 0-100
    if speed > 0:
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)
    elif speed < 0:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.HIGH)
    else:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(duty_cycle)

# Define encoder callback function
def encoder_callback(channel):
    global encoder_position
    a_value = GPIO.input(encoder_pin_a)
    b_value = GPIO.input(encoder_pin_b)
    if a_value == b_value:
        encoder_position += 1
    else:
        encoder_position -= 1

# Define debounce function for encoder interrupts
def debounce(channel):
    global encoder_position
    a_value = GPIO.input(encoder_pin_a)
    b_value = GPIO.input(encoder_pin_b)
    if a_value == b_value:
        encoder_position += 1
    else:
        encoder_position -= 1

debounce_time = 0.05  # Adjust debounce time as needed
GPIO.add_event_detect(encoder_pin_a, GPIO.BOTH, callback=debounce, bouncetime=int(debounce_time*1000))

# Main loop
try:
    while True:
        try:
            angle = float(input("Enter angle (-45 to 45 degrees): "))
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue

        # Limit angle to -45 to 45 degrees
        angle = max(-45, min(45, angle))

        # Turn to the specified angle
        ticks_per_degree = 10  # Adjust based on encoder resolution
        ticks = int(angle * ticks_per_degree)
        encoder_position = 0
        set_motor_speed(50 if angle > 0 else -50)

        # Wait for encoder to reach target position
        while abs(encoder_position) < abs(ticks):
            time.sleep(0.01)

        # Stop the motor
        set_motor_speed(0)

        # Return to zero position
        encoder_position = 0
        set_motor_speed(50 if encoder_position < 0 else -50)
        while abs(encoder_position) > 0:
            time.sleep(0.01)

        # Stop the motor
        set_motor_speed(0)

        # Wait for motor to stop
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up the GPIO resources when the program is terminated
    motor_pwm.stop()
    GPIO.cleanup()

    # Exit the program
    print("\nProgram terminated.")