import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

EncoderA = 2
EncoderB = 3

GPIO.setup(EncoderA,GPIO.IN)
GPIO.setup(EncoderB,GPIO.IN)

encoder_pos = 0
encoder_a_prev = GPIO.input(EncoderA)

def encoder_callback(channel):
    global encoder_pos
    global encoder_a_prev
    encoder_a = GPIO.input(EncoderA)
    encoder_b = GPIO.input(EncoderB)
    if encoder_a == encoder_a_prev:
        encoder_pos += (1 if encoder_b else -1)
    else:
        encoder_pos += (-1 if encoder_b else 1)
    encoder_a_prev = encoder_a

GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        print("Encoder position: ", encoder_pos)
        time.sleep(1)
finally:
    GPIO.cleanup()
