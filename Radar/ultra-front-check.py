import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the first sensor
trigger_pin1 = 20
echo_pin1 = 21

# Define GPIO pins for the second sensor
trigger_pin2 = 19
echo_pin2 = 26

GPIO.setup(trigger_pin1, GPIO.OUT)
GPIO.setup(echo_pin1, GPIO.IN)
GPIO.setup(trigger_pin2, GPIO.OUT)
GPIO.setup(echo_pin2, GPIO.IN)

def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)

    pulse_start = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    speed_of_sound = 343.2
    distance = pulse_duration * speed_of_sound / 2

    return distance

try:
    while True:
        distance1 = measure_distance(trigger_pin1, echo_pin1)
        distance2 = measure_distance(trigger_pin2, echo_pin2)

        print("Distance from Sensor 1: {:.2f} cm".format(distance1 * 100))
        print("Distance from Sensor 2: {:.2f} cm".format(distance2 * 100))

        time.sleep(1)
    print("t")
except KeyboardInterrupt:
    GPIO.cleanup()
