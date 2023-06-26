import RPi.GPIO as GPIO
import time

# Ultrasonic Sensor 1 GPIO pins
TRIG_PIN_1 = 20 # Right
ECHO_PIN_1 = 21

# Ultrasonic Sensor 2 GPIO pins
TRIG_PIN_2 = 4 # Left
ECHO_PIN_2 = 10

# Servo Motor 1 GPIO pin
SERVO_PIN_1 = 12

# Servo Motor 2 GPIO pin
SERVO_PIN_2 = 16

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN_1, GPIO.OUT)
GPIO.setup(ECHO_PIN_1, GPIO.IN)
GPIO.setup(TRIG_PIN_2, GPIO.OUT)
GPIO.setup(ECHO_PIN_2, GPIO.IN)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)
GPIO.setup(SERVO_PIN_2, GPIO.OUT)

# Create a PWM object for the servo motors
servo_pwm_1 = GPIO.PWM(SERVO_PIN_1, 50)  # 50 Hz frequency
servo_pwm_2 = GPIO.PWM(SERVO_PIN_2, 50)  # 50 Hz frequency

# Function to set the angle of the servo motor
def set_servo_angle(servo_pwm, angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    duty_cycle = angle / 18.0 + 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)


# Function to get distance from ultrasonic sensor
def get_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

try:
    # Initialize the servo motors
    servo_pwm_1.start(0)
    servo_pwm_2.start(0)
    set_servo_angle(servo_pwm_1, 0)  # Set the initial angle to 0 degrees
    set_servo_angle(servo_pwm_2, 0)  # Set the initial angle to 0 degrees

    # Perform scanning
    direction = 1  # 1 for forward, -1 for backward
    angle = 0
    # for i in range(21):
    while True:
        if angle >= 180:
            direction = -1  # Change direction to move backward
        elif angle <= 90:
            direction = 1   # Change direction to move forward

        angle += direction * 8  # Increment or decrement the angle by 10 degrees
        set_servo_angle(servo_pwm_1, angle)
        set_servo_angle(servo_pwm_2, angle)  # Reverse angle for the second servo
        time.sleep(0.05)  # Delay for the servos to reach the desired angle

        dist_1 = get_distance(TRIG_PIN_1, ECHO_PIN_1) # Rght
        dist_2 = get_distance(TRIG_PIN_2, ECHO_PIN_2) # Left
        print(f"Angle: {angle} | Distance 1: {dist_1} cm | Distance 2: {dist_2} cm")
        # time.sleep(0.03)


except KeyboardInterrupt:
    servo_pwm_1.stop()
    servo_pwm_2.stop()
    GPIO.cleanup()
