import RPi.GPIO as GPIO
import time
import collections

# Use the BCM GPIO numbers as the numbering scheme
GPIO.setmode(GPIO.BOARD)

# Use GPIO17 (Pin #11) as input, set pull-up resistance
GPIO.setup(29, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set initial values
rotation_count = 0
total_rotation_count = 0
start_time = time.time()
rpm_queue = collections.deque(maxlen=5)

def rotation_detection(channel):
    global rotation_count
    global total_rotation_count
    rotation_count += 1
    total_rotation_count += 1

# Use the falling edge detection to call a function
GPIO.add_event_detect(29, GPIO.FALLING, callback=rotation_detection, bouncetime=500)

try:
    while True:
        time.sleep(1)
        rotations_per_minute = (rotation_count / (time.time() - start_time)) * 60
        rpm_queue.append(rotations_per_minute)
        average_rpm = sum(rpm_queue) / len(rpm_queue)
        print(f'Average RPM over last 5 seconds: {average_rpm}')
        rotation_count = 0
        start_time = time.time()

except KeyboardInterrupt:
    total_runtime_minutes = (time.time() - start_time) / 60
    overall_average_rpm = total_rotation_count / total_runtime_minutes
    print(f'Total rotations: {total_rotation_count}')
    print(f'Average RPM over entire runtime: {overall_average_rpm}')
    GPIO.cleanup()
