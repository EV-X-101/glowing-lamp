import RPi.GPIO as GPIO
import time
from simple_pid import PID
import smbus2

# Pin definitions
MotorIn1 = 29     # Connect to IN1 on L298N
MotorIn2 = 31     # Connect to IN2 on L298N
MotorEnable = 33  # Connect to ENA on L298N

# ADS1115 parameters
ADS1115_ADDRESS = 0x48  # Address of the ADS1115, change if different
BUS_NO = 1              # I2C bus number, change if different
REGISTER_ADDRESS = 0x00 # Register to read data from
CONVERSION_REGISTER = 0x01

bus = smbus2.SMBus(BUS_NO)

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MotorIn1, GPIO.OUT)
GPIO.setup(MotorIn2, GPIO.OUT)
GPIO.setup(MotorEnable, GPIO.OUT)

# Set the motor direction
GPIO.output(MotorIn1, GPIO.HIGH)
GPIO.output(MotorIn2, GPIO.LOW)

# Set PWM parameters
Frequency = 50
DutyCycle = 20

# Set up PWM
motor = GPIO.PWM(MotorEnable, Frequency)
motor.start(DutyCycle)

# Initialize PID
# pid = PID(1.0, 0.1, 0.05)  # Kp=1.0, Ki=0.0, Kd=0.0
# pid.setpoint = 90
# print(pid(100))  # Should print -10.0 (since Kp=1.0 and the error is -10)

pid = PID(1.0, 0.1, 0.05,)
pid.output_limits = (0, 100)  # Output value will be between 0 and 100

def read_current_angle():
    # Write configuration to start conversion
    bus.write_i2c_block_data(ADS1115_ADDRESS, 0x01, [0xc3, 0x83])

    # Wait for conversion to complete
    time.sleep(0.1)

    # Read raw ADC value
    raw_adc = bus.read_i2c_block_data(ADS1115_ADDRESS, 0x00, 2)
    raw_adc = (raw_adc[0] << 8) | raw_adc[1]

    # Convert raw ADC value to voltage
    voltage = (raw_adc / 32767.0) * 4.096

    # Convert voltage to angle
    angle = (voltage / 3.3) * 180.0  # Assuming 0V = 0 degrees and 3.3V = 180 degrees

    return angle

try:
    while True:
        desired_angle = float(input("Enter the desired angle (0-180): "))
        pid.setpoint = int(desired_angle)

        while True:
            current_angle = read_current_angle()
            # control_value = pid(current_angle)
            error = pid.setpoint - current_angle
            control_value = pid(current_angle)
            # print(pid(100))  # Try some different values here
            print(f"Current angle: {current_angle}, Error: {error}, Control value: {control_value}")
            # print(f'Current angle: {current_angle}  Control: {control_value}')

            # if current_angle < desired_angle:  # Rotate right
            #     print(f'current_angle<desired_angle')
            #     GPIO.output(MotorIn1, 1)
            #     GPIO.output(MotorIn2, 0)

            # else:  # Rotate left
            #     GPIO.output(MotorIn1, 0)
            #     GPIO.output(MotorIn2, 1)
            #     print('Mee')

            # if desired_angle < 90:  # Rotate left
            #     GPIO.output(MotorIn1, GPIO.LOW)
            #     GPIO.output(MotorIn2, GPIO.HIGH)
            # elif desired_angle > 90:  # Rotate right
            #     GPIO.output(MotorIn1, GPIO.HIGH)
            #     GPIO.output(MotorIn2, GPIO.LOW)
            # else:  # Stop
            #     GPIO.output(MotorIn1, GPIO.LOW)
            #     GPIO.output(MotorIn2, GPIO.LOW)

            # Write output value to motor
            motor.ChangeDutyCycle(control_value)

            # Wait before next loop iteration
            time.sleep(0.01)

except KeyboardInterrupt:
    print("User interrupted, stopping.")

# Stop motor and cleanup GPIO
motor.stop()
GPIO.cleanup()
