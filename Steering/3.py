from simple_pid import PID
import time

pid = PID(5, 1, 0.01, setpoint=0)
pid.output_limits = (-1, 1)

current_position = 0

while True:
    target_position = float(input("Enter the target position: "))
    pid.setpoint = target_position
    while True:
        control_output = pid(current_position)
        print("Control output: ", control_output)
        if abs(current_position - target_position) <= 1:  # If we've reached the target
            break
        current_position += control_output  # Simulate the motor moving
        time.sleep(0.02)
