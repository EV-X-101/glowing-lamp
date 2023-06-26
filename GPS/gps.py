import serial
import time
# Configure serial communication
ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)

while True:
    # Read GPS data
   
    data = ser.readline().decode('utf-8', errors='ignore')
    # Check for valid NMEA sentence
    if data.startswith('$GPGGA'):
        parts = data.split(',')

        # Extract latitude, longitude, and other relevant information
        latitude = parts[2]
        longitude = parts[4]
        altitude = parts[9]

        # Print the GPS data
        print(f"Latitude: {latitude}\nLongitude: {longitude}\nAltitude: {altitude}")
ser.close()
