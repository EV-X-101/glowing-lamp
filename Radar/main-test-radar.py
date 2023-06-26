import RPi.GPIO as GPIO
import time
import math

# Ultrasonic Sensor GPIO pins
TRIG_PIN = 4
ECHO_PIN = 10

# Servo Motor GPIO pin
SERVO_PIN = 12

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create a PWM object for the servo motor
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency

# Function to set the angle of the servo motor
def set_servo_angle(angle):
    duty_cycle = (angle / 18) + 2
    servo_pwm.ChangeDutyCycle(duty_cycle)

# Function to get distance from ultrasonic sensor
def get_distance():
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

try:
    # Initialize the servo motor
    servo_pwm.start(0)
    set_servo_angle(0)  # Set the initial angle to 0 degrees

    # Perform scanning
    direction = 1  # 1 for forward, -1 for backward
    angle = 0
    while True:
        if angle >= 180:
            direction = -1  # Change direction to move backward
        elif angle <= 0:
            direction = 1   # Change direction to move forward

        angle += direction * 12  # Increment or decrement the angle by 10 degrees
        set_servo_angle(angle)
        time.sleep(0.1)  # Delay for the servo to reach the desired angle
        dist = get_distance()
        print(f"Angle: {angle} | Distance: {dist} cm")
        time.sleep(0.1)



except KeyboardInterrupt:
    servo_pwm.stop()
    GPIO.cleanup()

