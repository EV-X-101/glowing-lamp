import serial
import time
import socket

# Create a serial connection (adjust '/dev/ttyS0' and the baud rate as needed)
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

sock = socket.socket()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('192.168.0.77', 5000))
sock.listen()
print('Socket connection established.')

def convert_to_degrees(raw_value):
    decimal_value = float(raw_value)
    degrees = int(decimal_value / 100)
    d = decimal_value - degrees * 100
    return degrees + (d / 60)

while True:
    # Read one line from the serial buffer
    line = ser.readline()

    # Strip leading/trailing white space and convert to string
    line = line.strip().decode('utf-8', errors='ignore')
    
    # Ignore empty lines
    if line == '':
        continue

    # Ignore lines that don't start with '$GPRMC' - this is the start of the useful data
    if not line.startswith('$GPRMC'):
        continue

    # Split the line into a list of strings
    parts = line.split(',')

    # Extract latitude, longitude, and speed
    # latitude = convert_to_degrees(parts[3])
    # longitude = convert_to_degrees(parts[5])
    # speed = parts[7]

    # Latitude and longitude directions could also be crucial in some cases
    lat_dir = parts[4]
    long_dir = parts[6]

    # Changing latitude and longitude to negative if direction is South or West
    if lat_dir == "S":
        latitude = -latitude

    if long_dir == "W":
        longitude = -longitude

    try:
        # Send the GPS data to the PC over the socket connection
        sock.send(f'Latitude: {39.23435}, Longitude: {9.87562}, Speed (knots): {2.0}\n'.encode())
    except BrokenPipeError:
        print("Connection lost, attempting to reconnect")
        # Close the current socket
        sock.close()
        # Create a new socket object
        # sock = socket.socket()
        # Attempt to reconnect
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.bind(('0.0.0.0', 5000))
                sock.listen()
                print("Reconnected successfully")
                break
            except ConnectionRefusedError:
                print("Failed to reconnect, retrying in 5 seconds")
                time.sleep(5)

    # Print the values
    # print('Latitude:', latitude)
    # print('Longitude:', longitude)
    # print('Speed (knots):', speed)

    # Wait for a short period of time before reading the next line
    time.sleep(0.1)

# Don't forget to close the connection when you're done
sock.close()




