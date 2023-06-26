import serial
import time

# Create a serial connection (adjust '/dev/ttyS0' and the baud rate as needed)
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

def convert_to_degrees(raw_value):
    decimal_value = float(raw_value)
    degrees = int(decimal_value / 100)
    d = decimal_value - degrees * 100
    return degrees + (d / 60)

while True:
    # Read one line from the serial buffer
    line = ser.readline()

    # Strip leading/trailing white space and convert to string
    line = line.strip().decode('utf-8')
    
    # Ignore empty lines
    if line == '':
        continue

    # Ignore lines that don't start with '$GPRMC' - this is the start of the useful data
    if not line.startswith('$GPRMC'):
        continue

    # Split the line into a list of strings
    parts = line.split(',')

    # Extract latitude, longitude, and speed
    latitude = convert_to_degrees(parts[3])
    longitude = convert_to_degrees(parts[5])
    speed = parts[7]

    # Latitude and longitude directions could also be crucial in some cases
    lat_dir = parts[4]
    long_dir = parts[6]

    # Changing latitude and longitude to negative if direction is South or West
    if lat_dir == "S":
        latitude = -latitude

    if long_dir == "W":
        longitude = -longitude

    # Print the values
    print('Latitude:', latitude)
    print('Longitude:', longitude)
    print('Speed (knots):', speed)

    # Wait for a short period of time before reading the next line
    time.sleep(0.1)
