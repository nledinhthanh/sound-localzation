import time
import numpy as np
import sys
from daqhats import hat_list, HatIDs, mcc118
import RPi.GPIO as GPIO
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLC
import subprocess

# Declare microphone_positions and sound_speed variables
# Initialize necessary variables
microphone_positions = [(x, y ) for x, y  in [(0, 16), (8,0), (-8,0 )]]
sound_speed = 343.2
servo_min_angle = 0
servo_max_angle = 180

# Define clap_detection function
def clap_detection():
    result = subprocess.run(['python', 'clap_detection.py'], capture_output=True, text=True)
    is_clap_detected = result.stdout.strip() == 'True'
    return is_clap_detected

# Define gcc_subprocess function
def gcc_subprocess():
    result = subprocess.run(['python', 'gcc_subprocess.py'], capture_output=True, text=True)
    t1, t2, t3 = map(float, result.stdout.strip().split(','))
    return t1, t2, t3

# Define tdoa function
def tdoa(t1, t2, t3, microphone_positions, sound_speed):
    result = subprocess.run(['python', 'tdoa.py', str(t1), str(t2), str(t3)], capture_output=True, text=True)
    tdoa_distance, tdoa_angle = map(float, result.stdout.strip().split(','))
    return tdoa_distance, tdoa_angle

# Define servo_running function
def servo_running(angle):
    subprocess.run(['python', 'servo_running.py', str(angle)])

# Define lcd_distance function
def lcd_distance(gcc_distance):
    subprocess.run(['python', 'lcd_distance.py', str(gcc_distance)])

# Main function
def main():
    while True:
        if clap_detection:
            # Call GCC subprocess to get time delays
            t1, t2, t3 = gcc_subprocess()

            # Call TDOA subprocesses to calculate distance and angle
            tdoa_distance, tdoa_angle = tdoa(t1, t2, t3, microphone_positions, sound_speed)
            # Call servo running subprocess to move the servo motor
            servo_running(angle)

            # Call LCD subprocess to update distance information
            lcd_distance(gcc_distance)

            # Wait for a short period before restarting the loop
            time.sleep(0.001)
        else:
            time.sleep(0.0001)
if __name__ == "__main__":
    main()

