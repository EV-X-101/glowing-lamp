import RPi.GPIO as GPIO
import time

data_pin = 2
clk_pin = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(data_pin, GPIO.IN)
GPIO.setup(clk_pin, GPIO.IN)

def get_rotation():
    state = (GPIO.input(data_pin), GPIO.input(clk_pin))
    if state == (0, 1):
        return -1
    elif state == (1, 0):
        return 1
    else:
        return 0

def reset_position(channel):
    global position
    position = 0

def rotation_callback(channel):
    global position
    rotation = get_rotation()
    position += rotation
    print("Position:", position)

# Set up interrupt for reset
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(4, GPIO.FALLING, callback=reset_position, bouncetime=200)

# Set up interrupt for rotation detection
GPIO.add_event_detect(data_pin, GPIO.BOTH, callback=rotation_callback)

position = 0

try:
    while True:
        time.sleep(0.1)  # Keep the program running

finally:
    GPIO.cleanup()
