import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from simple_pid import PID

# Motor driver pins
pinMotorAForwards = 29
pinMotorABackwards = 31
pinMotorAPWM = 33

# Set the GPIO Pin mode
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(pinMotorAForwards, GPIO.OUT)
GPIO.setup(pinMotorABackwards, GPIO.OUT)
GPIO.setup(pinMotorAPWM, GPIO.OUT)

# Set PWM parameters
Frequency = 1000
DutyCycle = 100

# Set up PWM
MotorAPWM = GPIO.PWM(pinMotorAPWM, Frequency)
MotorAPWM.start(DutyCycle)

# PID controller parameters
Kp = 1
Ki = 0.5
Kd = 0.05

#(1, 0.1, 0.05)

# Ask the user for the desired angle
desired_angle = int(input("Please enter the desired angle (0-108): "))

# Create the PID controller
pid = PID(Kp, Ki, Kd, setpoint=desired_angle)
pid.output_limits = (-1, 1)  # Output value will be clamped between -1 and 1

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

def map_range(value, left_min, left_max, right_min, right_max):
    # Map the input range to the output range
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

try:
    while True:
        raw_value = chan.value
        current_angle = int(map_range(raw_value, 0, 65535, 0, 270))  # 270 degrees full scale for the potentiometer
        control = pid(current_angle)
        #print(f'Current angle: {current_angle}    Control: {control}')

        if control > 0:
            GPIO.output(pinMotorAForwards, 1)
            GPIO.output(pinMotorABackwards, 0)
        
        else:
            GPIO.output(pinMotorAForwards, 0)
            GPIO.output(pinMotorABackwards, 1)

    MotorAPWM.ChangeDutyCycle(abs(control) * 100)
    time.sleep(0.01)

except KeyboardInterrupt:
    MotorAPWM.stop()
    GPIO.cleanup()


    


    
    
  