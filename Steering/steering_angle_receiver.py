# import serial
# import time

# # Open the serial port that the Arduino is connected to.
# ser = serial.Serial('/dev/ttyACM0', 9600)

# # Wait for the Arduino to initialize.
# time.sleep(2)

# while True:
#     if ser.in_waiting > 0:
#         try:
#             line = ser.readline().decode('utf-8').rstrip()
#             print(line)
#         except UnicodeDecodeError:
#             pass  # Ignore lines that can't be decoded as UTF-8.

import serial
import time

# Create a serial connection (replace '/dev/ttyACM0' with your Arduino's port)
ser = serial.Serial('/dev/ttyACM0', 9600)

# Wait for the Arduino to initialize
time.sleep(2)

turn = {'2': 'left', '1': 'right', '0': 'center'}

try:
    while True:
        # Ask the user for a command
        command = input("Enter command: ")

        # Check if the command is valid
        if command not in turn:
            print("Invalid command. Please enter '2', '1', or '0'.")
            continue

        # Send the command to the Arduino
        ser.write((turn[command] + "\n").encode())
        print(f'Sent command: {command}')

        # Wait for response from Arduino and print it
        while ser.in_waiting > 0:
            print("Response from Arduino: " + ser.readline().decode('utf-8', errors='ignore').strip())

        # Wait a bit before sending the next command
        time.sleep(2)
except KeyboardInterrupt:
    # Close the serial connection
    ser.close()
    print("Serial connection closed.")
