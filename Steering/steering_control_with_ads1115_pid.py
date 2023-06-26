import time
# import board
# import busio
import RPi.GPIO as GPIO
# import adafruit_ads1x15.ads1115 as ADS
# from adafruit_ads1x15.analog_in import AnalogIn
from simple_pid import PID
import smbus

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
Frequency = 30
DutyCycle = 20

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

# ADS1115 parameters
ADS1115_ADDRESS = 0x48  # Address of the ADS1115, change if different
ADS1115_CHANNEL = 0     # Channel on the ADS1115 that the potentiometer is connected to
BUS_NO = 1              # I2C bus number, change if different
REGISTER_ADDRESS = 0x00 # Register to read data from

bus = smbus.SMBus(BUS_NO)

# Create the I2C bus
# i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
# ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
# chan = AnalogIn(ads, ADS.P0)

def read_current_angle():
    # Read raw ADC value
    raw_adc = bus.read_word_data(ADS1115_ADDRESS, REGISTER_ADDRESS + ADS1115_CHANNEL)

    # Convert raw ADC value to voltage
    voltage = (raw_adc * 3.3) / 32768.0  # Assuming a 3.3V reference voltage

    # Convert voltage to angle
    angle = (voltage / 3.3) * 180.0  # Assuming 0V = 0 degrees and 3.3V = 180 degrees

    return angle

'''def map_range(value, left_min, left_max, right_min, right_max):
    # Map the input range to the output range
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)'''

try:
    while True:
        # raw_value = chan.value
        # current_angle = int(map_range(raw_value, 0, 65535, 0, 270))  # 270 degrees full scale for the potentiometer
        current_angle = read_current_angle()
        control = pid(current_angle)
        print(f'Current angle: {current_angle}    Control: {control}')

        if control > 0:
            print('Hello')
            GPIO.output(pinMotorAForwards, 1)
            GPIO.output(pinMotorABackwards, 0)

        else:
            print('Mee')
            GPIO.output(pinMotorAForwards, 0)
            GPIO.output(pinMotorABackwards, 1)

        MotorAPWM.ChangeDutyCycle(abs(control) * 100)
        time.sleep(0.01)

except KeyboardInterrupt:
    MotorAPWM.stop()
    GPIO.cleanup()


    


    
    
  