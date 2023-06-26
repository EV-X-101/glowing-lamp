import RPi.GPIO as GPIO

# GPIO pin numbers connected to the rotary encoder
PIN_A = 35
PIN_B = 37

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to track the rotation
rotation_count = 0
rotation_degree = 0

# Define a callback function for detecting encoder rotation
def encoder_callback(channel):
    global rotation_count, rotation_degree

    # Read the state of the pins
    pin_a_state = GPIO.input(PIN_A)
    pin_b_state = GPIO.input(PIN_B)

    # Update the rotation count and degree based on the pin states
    if pin_a_state == pin_b_state:
        rotation_count += 1
    else:
        rotation_count -= 1

    # Calculate the rotation degree (assuming 360-degree rotation)
    rotation_degree = (rotation_count % 360)

# Register the callback for the A pin, detect both rising and falling edges
GPIO.add_event_detect(PIN_A, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        # Print the current rotation degree
        print("Rotation Degree:", rotation_degree)

except KeyboardInterrupt:
    # Clean up GPIO on program exit
    GPIO.cleanup()
