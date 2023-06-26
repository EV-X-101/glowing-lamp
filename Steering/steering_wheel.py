import RPi.GPIO as GPIO
import time


# Set mode and pins
GPIO.setmode(GPIO.BOARD)

# Motor 1 pins
in1 = 29
in2 = 31
en1 = 33

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)
# Set up PWM
p1 = GPIO.PWM(en1,2000)
p1.start(0)

# Map steering angle and direction to motor speed
def map_steering_to_speed(steering_angle, steering_direction):
    # Calibration constants
    MIN_ANGLE = -90    # minimum steering angle in degrees
    MAX_ANGLE = 90     # maximum steering angle in degrees
    MIN_SPEED = 20     # minimum motor speed in percent
    MAX_SPEED = 100    # maximum motor speed in percent

    # Calculate the motor speed based on the steering angle and direction
    if steering_direction == "left":
        speed_left = 0
        speed_right = abs(steering_angle) * 100 / 90 #abs(steering_angle) * (MAX_SPEED - MIN_SPEED) / (MAX_ANGLE - MIN_ANGLE) + MIN_SPEED
    elif steering_direction == "right":
        speed_left = abs(steering_angle) * 100 / 90 # abs(steering_angle) * (MAX_SPEED - MIN_SPEED) / (MAX_ANGLE - MIN_ANGLE) + MIN_SPEED
        speed_right = 0
    else:
        speed_left = 0
        speed_right = 0

    # Limit the motor speed to the calibration range
    speed_left = max(min(speed_left, MAX_SPEED), MIN_SPEED)
    speed_right = max(min(speed_right, MAX_SPEED), MIN_SPEED)

    return speed_left, speed_right

def turn_to(angle):
    if angle>=1:
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        p1.ChangeDutyCycle(abs(angle))
    elif angle<=1:
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        p1.ChangeDutyCycle(abs(angle))
    
    else:
        pass

# Main loop
while True:
    # Read steering angle and direction from terminal in one line
    while True:
        try:
            steering_angle, steering_direction = input("Enter steering angle (-90 to 90 degrees) and direction (left or right), separated by a comma: ").split(",")
            steering_angle = float(steering_angle)
            if steering_angle < -90 or steering_angle > 90:
                raise ValueError("Steering angle must be between -90 and 90 degrees")
            if steering_direction not in ["left", "right"]:
                raise ValueError("Steering direction must be 'left' or 'right'")
            break
        except ValueError as e:
            print("Invalid input: ", e)

    # Map steering angle and direction to motor speed
    speed_left, speed_right = map_steering_to_speed(steering_angle, steering_direction)

    # Set motor speed
    print(speed_left, "left")
    print(speed_right, "right")
    turn_to(steering_angle)