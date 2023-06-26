import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

def map_range(value, left_min, left_max, right_min, right_max):
    # map the input range to the output range
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

def measured_angle():
    while True:
        raw_value = chan.value
        print(raw_value)
        # assuming the potentiometer is 270 degree type
        # mapping the raw value (0-65535 for ADS1115) to angle (0-270)
        angle = map_range(raw_value, 0, 65535, 0, 270)
        # print('Raw ADC Value: {0}\tMapped angle: {1:0.1f}'.format(raw_value, int(angle)))
        # time.sleep(0.5)
        print(angle)
        # return angle


measured_angle()