import RPi.GPIO as GPIO
import time


# Pin Definitions
motor_pin1 = 29
motor_pin2 = 31
enable_pin = 33
ENC_A = 35
ENC_B = 37

# Set up GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Initialize variables
counter = 0
a_state = GPIO.input(ENC_A)
b_state = GPIO.input(ENC_B)

# Set up PWM for motor speed control
motor_pwm = GPIO.PWM(enable_pin, 10000)
motor_pwm.start(0)


# Define callback function for encoder interrupt
def encoder_callback(channel):
    global counter, a_state, b_state
    a_next_state = GPIO.input(ENC_A)
    b_next_state = GPIO.input(ENC_B)
    if a_next_state != a_state:
        counter += 1 if a_next_state == b_next_state else -1
        print("Pulse: {}, Ticks: {}".format(a_next_state, counter))
    a_state, b_state = a_next_state, b_next_state


print(counter, 'wowowowwowowowoooo')
# Define motor control functions
def set_motor_speed(speed):
    if speed > 0:
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)
        #print("+++++")
    elif speed < 0:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.HIGH)
        #print("-----")
    else:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.LOW)

    motor_pwm.ChangeDutyCycle(abs(speed))


# Set up encoder interrupt



def turn_to(angle):
    global counter
    # Turn to the specified angle
    
    print('Turnning to the specified angle', counter )
    while abs(counter) < round(abs(angle)/3):
        #print(round(angle/9),counter)
        set_motor_speed(100 if angle > 0 else -100)
        # print(round(abs(angle)/3),counter, '((|))')
     # Stop the motor
    time.sleep(5)
    print('done')
    set_motor_speed(0)
    # Wait for motor to stop
    time.sleep(abs(angle) / 45.0)


def return_to_o():
    global counter
    # Return to zero position
    print('Returning to zero position')
    while counter == 0:
        set_motor_speed(-100 if angle > 0 else 100)
        print(counter, '------')
     # Stop the motor
    set_motor_speed(0)
    # Wait for motor to stop
    time.sleep(abs(angle) / 45.0)



# Set up encoder interrupt
GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=encoder_callback)

# Main loop
while True:
    # Accept user input
    while True:
        try:
            
            print(counter,'Printing counter')
            angle = float(input("Enter angle (-45 to 45 degrees): "))
            # Limit angle to -45 to 45 degrees
            angle = max(-45, min(45, angle))
            turn_to(angle)
            return_to_o()
            break
        except ValueError:
            print("Invalid input. Please enter a number.")

    
    

    
    


# Clean up the GPIO resources when the program is terminated


motor_pwm.stop()
GPIO.cleanup()

# Exit the program
print("\nProgram terminated.")

