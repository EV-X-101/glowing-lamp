import RPi.GPIO as GPIO
import time
from simple_pid import PID  # You'll need to install this library

# Setup the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define pins
clk = 35
dt = 37
ena = 33  # ENA pin on L298N
in1 = 29  # IN1 pin on L298N
in2 = 31  # IN2 pin on L298N

# Setup the pins
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
pwm = GPIO.PWM(ena, 100)  # PWM at 100Hz
pwm.start(0)  # Initial duty cycle

# Initialize PID controller
# You will need to tune these parameters based on your specific system
pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid.output_limits = (-100, 100)  # Output value will be clamped between these limits

# Initialize encoder state
counter = 0
clkLastState = GPIO.input(clk)

desired_angle = float(input("Enter desired angle (0-180): "))

try:
    while True:
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        if clkState != clkLastState:
            if dtState != clkState:
                counter += 1
            else:
                counter -= 1
        clkLastState = clkState
        print(dtState, clkState)
        time.sleep(0.01)

        # Calculate current angle assuming full rotation gives a count of 100
        # This will depend on your specific encoder
        current_angle = counter / 100 * 360  # scale counter to 0-360 range
        
        # Compute the control value using the PID controller
        pid.setpoint = desired_angle
        control = pid(current_angle)
        print(control)
        
        
        # Convert the control value to duty cycle and direction
        if control >= 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        
        pwm.ChangeDutyCycle(abs(control))
finally:
    pwm.stop()
    GPIO.cleanup()

