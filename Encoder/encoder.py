import RPi.GPIO as GPIO

# Pin definitions
ENC_A = 35
ENC_B = 37

# Initialize GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize variables
counter = 0
a_state = GPIO.input(ENC_A)
b_state = GPIO.input(ENC_B)


print('-----------------------')
# Define callback function for encoder interrupt
def encoder_callback(channel):
    global counter, a_state, b_state
    a_next_state = GPIO.input(ENC_A)
    b_next_state = GPIO.input(ENC_B)
    if a_next_state != a_state:
        counter += 1 if a_next_state == b_next_state else -1
        print("Pulse: {}, Ticks: {}".format(a_next_state, counter))
    a_state, b_state = a_next_state, b_next_state

# Set up encoder interrupt
GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=encoder_callback)

# Main loop
try:
    while True:
        pass
except KeyboardInterrupt:
    GPIO.cleanup()