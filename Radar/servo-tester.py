import RPi.GPIO as GPIO
import time

# Servo Motor GPIO pin
SERVO_PIN = 16

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create a PWM object for the servo motor
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency

# Function to set the angle of the servo motor
def set_servo_angle(angle):
    duty_cycle = (angle / 18) + 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)

try:
    # Initialize the servo motor
    servo_pwm.start(0)

    # Test the rotation of the servo motor
    for angle in range(0, 181, 10):
        set_servo_angle(angle)
        time.sleep(0.5)

except KeyboardInterrupt:
    servo_pwm.stop()
    GPIO.cleanup()
