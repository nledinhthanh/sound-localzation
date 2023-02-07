#This code using all available library in python to calculate the distance and direction of the source
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt

# Define the sample rate and number of samples
fs = 44100
num_samples = 44100

# Define the microphone positions as an array
mic_pos = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0]])

# Function to calculate the direction of the source and distance of the sound source
def get_direction_and_distance(data):
    # Calculate the cross-correlation between each microphone pair
    R = np.corrcoef(data)
    
    # Calculate the eigenvectors and eigenvalues of the correlation matrix
    eig_vals, eig_vecs = np.linalg.eig(R)
    
    # The direction is given by the eigenvector corresponding to the maximum eigenvalue
    max_index = np.argmax(eig_vals)
    direction = np.rad2deg(np.arctan2(eig_vecs[1, max_index], eig_vecs[0, max_index]))
    
    # Calculate the distances between the sound source and each microphone
    distances = np.zeros(3)
    for i in range(3):
        distances[i] = np.linalg.norm(mic_pos[i] - eig_vecs[:, max_index])
    
    # The sound source distance is given by the average of the distances to each microphone
    distance = np.mean(distances)
    
    return direction, distance

# Record audio from each microphone
def record_audio():
    # Create a list to store the audio data from each microphone
    data = []
    
    # Record audio from each microphone
    for i in range(3):
        mic_data = sd.rec(num_samples, fs, channels=1)
        data.append(mic_data[:, 0])
        
    return data

def visualize_direction_and_distance(direction, distance):
    plt.figure()
    plt.title("Sound Source Direction and Distance")
    plt.xlabel("Direction of Arrival (Degrees)")
    plt.ylabel("Distance (m)")
    plt.scatter(direction, distance, color='red')
    plt.show() 

# Main function to detect sound source direction and distance
def detect_sound_source_direction_and_distance():
    data = record_audio()
    direction, distance = get_direction_and_distance(data)
    print("Direction of the source compare to center point: ", direction)
    print("Distance of the source compare to center point : ", distance)

if __name__ == '__main__':
    detect_sound_source_direction_and_distance()
    visualize_direction_and_distance(direction, distance)

