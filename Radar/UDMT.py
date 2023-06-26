# Ultrasonic Distance Measure Test.
import RPi.GPIO as GPIO
import time

# Set the GPIO pin numbers for TRIG and ECHO
TRIG_PIN = 20
ECHO_PIN = 21

# Set the GPIO mode to use BCM pin numbers
GPIO.setmode(GPIO.BCM)

# Set up the TRIG and ECHO pins as outputs and inputs, respectively
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    # Send a 10us pulse to the TRIG pin to start a measurement
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Wait for the ECHO pin to go high and then low again
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    # Calculate the duration of the pulse and the distance in centimeters
    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 34300 / 2

    # Return the distance measurement
    return distance_cm

# Main loop
while True:
    # Get the distance measurement and print it to the console
    distance = get_distance()
    print(f"Distance: {distance:.2f} cm")

    # Wait a short time before taking another measurement
    time.sleep(0.1)
