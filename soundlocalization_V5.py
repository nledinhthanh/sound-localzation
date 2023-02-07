#this version only trialateration base and print out to the 1602 LCD via I2C pin 
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import time
import sys
import smbus
import math

# Define the sample rate and number of samples
fs = 44100
num_samples = 44100

# Define the microphone positions as an array
mic_dis = 0.15 # the fixed distance between each microphone in the microphones array 
mic_pos = np.array([[0, 0], [mic_dis, 0], [mic_dis/2, np.sqrt(3)/2 * mic_dis]]) * 100

# Function to triangulation by using the multilateration give the estimated position of the source as the result
def multilateration(mic_positions, delta_distances):
    A = np.zeros((3, 2))
    b = np.zeros((3, 1))
    for i in range(3):
        for j in range(3):
            if i != j:
                diff = mic_positions[i] - mic_positions[j]
                A[i, :] += diff / np.linalg.norm(diff)
                b[i] -= delta_distances[j] / np.linalg.norm(diff)
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return x + mic_positions[0]

# Function to calculate the distance between 2 points
def PtoP_distance (p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# Funtion to refine the estimated position of the sound source using trilateration
def trilateration(es_position, mic_pos):

     #Distance between the estimated position of the source to each microphone 
    d1 = PtoP_distance (position, mic_pos[1])
    d2 = PtoP_distance (position, mic_pos[2])
    d3 = PtoP_distance (position, mic_pos[3])
    distances =np.array ([d1, d2, d3])

    # Compute the circles representing the loci of points equidistant from each microphone
    circles = np.zeros((3, 3))
    for i in range(3):
        circles[i, :2] = mic_positions[i]
        circles[i, 2] = distances[i]

    # Intersect the circles to obtain the two possible positions of the sound source
    ex1, ey1, ex2, ey2 = intersect_circles(circles[0], circles[1], circles[2])
    p1 = np.array([ex1, ey1])
    p2 = np.array([ex2, ey2])

    # Choose the position that is closest to the estimated position obtained from multilateration
    estimated_position = multilateration(mic_positions, distances)
    if np.linalg.norm(p1 - estimated_position) < np.linalg.norm(p2 - estimated_position):
        return p1
    else:
        return p2

# Function to calculate the direction and distance of the sound source using triangulation
def get_direction_and_distance_triangulation(data):
    # Calculate the time delays between the microphones
    delays = np.zeros(3)
    for i in range(3):
        delays[i] = np.argmax(data[i]) / fs
    
    # Calculate the differences in time delays
    diffs = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            diffs[i, j] = delays[i] - delays[j]
            
    # Calculate the differences in distances between the sound source and each pair of microphones 
    distances = np.zeros(3)
    d12 = diffs[1, 2] * 340
    d13 = diffs[1, 3] * 340
    d23 = diffs[2, 3] * 340
    delta_distances = np.array([d12, d13, d23])

    #Estimate the position of the source 
    es_position = multilateration(mic_pos, delta_distances)
    
   

    # Refining the position to make more accurate outcome
    position = trialateration (es_position, mic_pos)

    # Calculate the direction using the law of cosines
    cos_direction = np.zeros(3)
    for i in range(3):
        cos_direction[i] = np.dot(mic_pos[i], mic_pos[(i + 1) % 3]) / (np.linalg.norm(mic_pos[i]) * np.linalg.norm(mic_pos[(i + 1) % 3]))
    
    direction = np.arccos(np.mean(cos_direction))
    direction = np.rad2deg(direction)
    
    return direction, distance

# Callback function to detect the clap and record audio
def clap_detect_callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    global data
    global trigger
    global clap_detected
    
    # Check if the audio data exceeds a certain threshold to detect a clap
    if np.max(np.abs(indata[:, 0])) > 0.3:
        trigger = True
        
    if trigger:
        data.append(indata[:, 0])
        if len(data) == num_samples:
            clap_detected = True
            trigger = False

# Define the I2C bus and device address
bus = smbus.SMBus(1)
device_address = 0x27

# Define the function to send commands to the LCD
def lcd_send_command(command):
    bus.write_byte(device_address, 0x80)
    bus.write_byte(device_address, command)

# Define the function to send data to the LCD
def lcd_send_data(data):
    bus.write_byte(device_address, 0x40)
    bus.write_byte(device_address, data)

# Initialize the LCD
lcd_send_command(0x33)
lcd_send_command(0x32)
lcd_send_command(0x28)
lcd_send_command(0x0C)
lcd_send_command(0x06)
lcd_send_command(0x01)
 
# Main function to detect clap direction and distance
def main():
    global data
    global trigger
    global clap_detected
    # Initialize variables
    data = []
    trigger = False
    clap_detected = False

# Start recording audio from microphone
with sd.InputStream(channels=1, samplerate=fs, callback=clap_detect_callback):
    while not clap_detected:
        time.sleep(0.1)
        
# Convert the recorded data to a NumPy array
data = np.array(data)

# Calculate the direction and distance of the sound source
direction, distance = get_direction_and_distance_triangulation(data)

# Clear the LCD screen
lcd_send_command(0x01)

# Display the distance on the first line of the LCD screen
lcd_send_command(0x80)
lcd_send_data("Distance: {:.2f} cm".format(distance))

# Display the direction on the second line of the LCD screen
lcd_send_command(0xC0)
lcd_send_data("direction: {:.2f} deg".format(direction))


