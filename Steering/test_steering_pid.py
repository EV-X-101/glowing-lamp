
import RPi.GPIO as GPIO
import time
from simple_pid import PID

# Set up GPIO pins for motor driver
GPIO.setmode(GPIO.BOARD)
motor_pin_1 = 29
motor_pin_2 = 31
motor_en_pin = 33
GPIO.setup(motor_pin_1, GPIO.OUT)
GPIO.setup(motor_pin_2, GPIO.OUT)
GPIO.setup(motor_en_pin, GPIO.OUT)
motor_pwm = GPIO.PWM(motor_en_pin, 1000)
motor_pwm.start(0)

# Set up motor driver parameters
duty_cycle = 100

# Set up GPIO pins for encoder
encoder_pin_a = 35
encoder_pin_b = 37
GPIO.setup(encoder_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up variables for encoder state
encoder_state_a = GPIO.input(encoder_pin_a)
encoder_state_b = GPIO.input(encoder_pin_b)
encoder_last_state_a = encoder_state_a

# Set up variables for angle measurement
angle = 0
last_angle = 0

# Set up PID controller
Kp = 1.0
Ki = 0.0
Kd = 0.0
setpoint = 90.0
pid = PID(Kp, Ki, Kd, setpoint)

# Set up variables for timing
last_time = time.time()

# Set up function to handle encoder interrupts
def handle_encoder(pin):
    global encoder_state_a
    global encoder_state_b
    global encoder_last_state_a
    global angle
    global last_angle
    global last_time

    encoder_state_a = GPIO.input(encoder_pin_a)
    encoder_state_b = GPIO.input(encoder_pin_b)

    if encoder_state_a != encoder_last_state_a:
        if encoder_state_a == 0:
            if encoder_state_b == 1:
                angle += 1
            else:
                angle -= 1

    encoder_last_state_a = encoder_state_a

    current_time = time.time()
    if current_time - last_time > 0.1:
        last_angle = angle
        last_time = current_time

# Set up interrupt for encoder pin A
GPIO.add_event_detect(encoder_pin_a, GPIO.BOTH, callback=handle_encoder)

# Loop until user enters 'stop'
while True:
    command = input("Enter 'stop' to end the program: ")
    if command == 'stop':
        break

    # Compute motor speed using PID control
    speed = pid(last_angle)
    if speed > 0:
        GPIO.output(motor_pin_1, GPIO.HIGH)
        GPIO.output(motor_pin_2, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(duty_cycle * speed)
    else:
        GPIO.output(motor_pin_1, GPIO.LOW)
        GPIO.output(motor_pin_2, GPIO.HIGH)
        motor_pwm.ChangeDutyCycle(duty_cycle * abs(speed))

# Clean up GPIO pins
GPIO.cleanup()








# import time
# import math
# from simple_pid import PID
# import RPi.GPIO as GPIO

# # Initialize motor control pins
# motor_pin_1 = 29
# motor_pin_2 = 31
# motor_en_pin = 33
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(motor_pin_1, GPIO.OUT)
# GPIO.setup(motor_pin_2, GPIO.OUT)
# GPIO.setup(motor_en_pin, GPIO.OUT)
# motor_pwm = GPIO.PWM(motor_en_pin, 1000)
# motor_pwm.start(0)

# # Initialize rotary encoder
# gpio_a = 35
# gpio_b = 37
# encoder_ticks = 360
# GPIO.setup(gpio_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(gpio_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# encoder_count = 0
# encoder_last_a = GPIO.input(gpio_a)

# def encoder_callback(channel):
#     global encoder_count, encoder_last_a
#     a = GPIO.input(gpio_a)
#     b = GPIO.input(gpio_b)
#     if a != encoder_last_a:
#         if b != a:
#             encoder_count -= 1
#         else:
#             encoder_count += 1
#     encoder_last_a = a

# GPIO.add_event_detect(gpio_a, GPIO.BOTH, callback=encoder_callback)

# # Set motor and encoder to 0 position
# motor_pwm.ChangeDutyCycle(0)
# while encoder_count != 0:
#     if encoder_count > 0:
#         GPIO.output(motor_pin_1, GPIO.HIGH)
#         GPIO.output(motor_pin_2, GPIO.LOW)
#         motor_pwm.ChangeDutyCycle(50)
#     else:
#         GPIO.output(motor_pin_1, GPIO.LOW)
#         GPIO.output(motor_pin_2, GPIO.HIGH)
#         motor_pwm.ChangeDutyCycle(50)
#     time.sleep(0.01)
# motor_pwm.ChangeDutyCycle(0)

# # Setpoint for steering angle (radians)
# setpoint = 0.0

# # Loop until user enters "end"
# while True:
#     # Prompt user for target angle
#     target_angle_str = input("Enter target angle in degrees (-45 to 45, or 'end' to stop): ")
#     if target_angle_str == "end":
#         break
#     target_angle = math.radians(float(target_angle_str))
    
#     # Check if target angle is within range
#     if abs(math.degrees(target_angle)) > 45:
#         print("Target angle must be between -45 and 45 degrees.")
#         continue

#     # Loop until target angle is reached
#     while True:
#         # Read current steering angle from rotary encoder
#         current_angle = (2 * math.pi * encoder_count) / encoder_ticks

#         # Calculate PID output
#         # (code for PID controller goes here)
#         pid = PID(1, 0, 0, setpoint=target_angle)
#         pid.output_limits = (-100, 100)
#         pid_out = pid(target_angle)

#         # Convert PID output to motor control signal
#         if pid_out > 0:
#             GPIO.output(motor_pin_1, GPIO.HIGH)
#             GPIO.output(motor_pin_2, GPIO.LOW)
#             motor_pwm.ChangeDutyCycle(pid_out)
#         elif pid_out < 0:
#             GPIO.output(motor_pin_1, GPIO.LOW)
#             GPIO.output(motor_pin_2, GPIO.HIGH)
#             motor_pwm.ChangeDutyCycle(abs(pid_out))
#         else:
#             motor_pwm.ChangeDutyCycle(0)

#         # Check if target angle has been reached
#         error = abs(setpoint - current_angle)
#         if error < 0.01:
#             # Stop motor and print "Reached"
#             motor_pwm.ChangeDutyCycle(0)
#             print("Reached")
#             break
#         else:
#             # Print "Approaching"
#             print("Approaching")

#         # Wait for some time
#         time.sleep(0.1)

# # Cleanup GPIO
# GPIO.cleanup()






