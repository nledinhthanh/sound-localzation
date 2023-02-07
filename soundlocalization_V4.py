import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import time
import sys

# Define the sample rate and number of samples
fs = 44100
num_samples = 44100

# Define the microphone positions as an array
distance = 0.15
mic_pos = np.array([[0, 0, 0], [distance, 0, 0], [distance/2, np.sqrt(3)/2 * distance, 0]]) * 100

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
            
    # Calculate the distances between the sound source and each microphone
    distances = np.zeros(3)
    for i in range(3):
        for j in range(3):
            if i != j:
                distances[i] = np.linalg.norm(mic_pos[i] - mic_pos[j]) / (diffs[i, j] * 340)
                break
    
    # The sound source distance is given by the average of the distances to each microphone
    distance = np.mean(distances)
    
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
    print("Direction of the source compare to center point: ", direction)
    print("Distance of the source compare to center point : ", distance)

