import numpy as np
import sounddevice as sd
from scipy.signal import butter, lfilter
import paho.mqtt.client as mqtt
import json
import time

# DTMF frequencies
DTMF_FREQUENCIES = {
    (697, 1209): '1', (697, 1336): '2', (697, 1477): '3', (697, 1633): 'A',
    (770, 1209): '4', (770, 1336): '5', (770, 1477): '6', (770, 1633): 'B',
    (852, 1209): '7', (852, 1336): '8', (852, 1477): '9', (852, 1633): 'C',
    (941, 1209): '*', (941, 1336): '0', (941, 1477): '#', (941, 1633): 'D'
}

ROW_FREQUENCIES = [697, 770, 852, 941]
COLUMN_FREQUENCIES = [1209, 1336, 1477, 1633]

FREQ_TOLERANCE = 15
MIN_MAGNITUDE = 1  # Adjust this value based on sensitivity
DOMINANCE_RATIO = 1  # Detected frequency must be x times stronger than others
REQUIRED_CONFIRMATIONS = 5  # Number of confirmations to avoid false positives

route_mode = False
command_list = []
COMMAND_DURATION = 3  # Duration for each command in seconds

MQTT_SERVER = "localhost"  # Replace with your MQTT broker address
MQTT_PORT = 1883  # Replace with your MQTT broker port
MQTT_TOPIC = "mqtt_vel"  # Replace with your MQTT topic

# MQTT client setup
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_SERVER, MQTT_PORT, 60)
mqtt_client.loop_start()

# Globals for debouncing
last_detected = None
confirmation_count = 0
TONE_DELAY = 0.24
last_tone_time = 0
#Global var for checksum
expecting_checksum = False
total_sum = 0

def calculate_checksum(command_list):
    """
    Calculate a checksum using modulo 10 for the given command list.
    """
    checksum = sum(int(cmd) for cmd in command_list if cmd.isdigit()) % 9
    return checksum

def validate_checksum(received_checksum):
    """
    Validate the received checksum against the calculated checksum.
    """
    calculated_checksum = calculate_checksum(command_list)
    if calculated_checksum == received_checksum:
        print(f"Checksum valid: {received_checksum}")
        return True
    else:
        print(f"Checksum invalid: received {received_checksum}, calculated {calculated_checksum}")
        return False

def publish_movement_from_command(command):
    """
    Publish movement based on the number pad-like command.
    """
    if command == '7':
        publish_movement(0.2, 0.785)  # Forward and left
    elif command == '8':
        publish_movement(0.2, 0.0)  # Forward
    elif command == '9':
        publish_movement(0.2, -0.785)  # Forward and right
    elif command == '4':
        publish_movement(0.0, 1.571)  # Left
    elif command == '5':
        publish_movement(0.0, 0.0)  # Full stop
    elif command == '6':
        publish_movement(0.0, -1.571)  # Right
    elif command == '1':
        publish_movement(-0.2, 0.785)  # Backward and left
    elif command == '2':
        publish_movement(-0.2, 0.0)  # Backward
    elif command == '3':
        publish_movement(-0.2, -0.785)  # Backward and right

def execute_route():
    """
    Executes commands stored in the route plan sequentially.
    """
    global command_list
    print("Executing route...")
    for command in command_list:
        publish_movement_from_command(command)
        if command in ['2', '8']:
            time.sleep(3)  # Run for 3 seconds
        else:
            time.sleep(1)  # Run for 1 second
    print("Route execution completed.")
    command_list = []

def publish_movement(linear_x, angular_z):
    """
    Publish movement command via MQTT.
    """
    payload = {
        "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
    }
    mqtt_client.publish(MQTT_TOPIC, json.dumps(payload))
    print(f"Published to {MQTT_TOPIC}: {payload}")


def bandpass_filter(signal, lowcut, highcut, sample_rate, order=5):
    # Band-pass filter to isolate DTMF range.

    nyquist = 0.5 * sample_rate
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return lfilter(b, a, signal)


def goertzel(samples, sample_rate, target_freq):
    """
    Implements the Goertzel algorithm to detect a target frequency.

    Args:
        samples: Input signal samples.
        sample_rate: Sampling rate of the signal.
        target_freq: Frequency to detect.

    Returns:
        Power of the detected frequency.
    """
    n = len(samples)
    k = int(0.5 + (n * target_freq) / sample_rate)
    omega = (2.0 * np.pi * k) / n
    sine = np.sin(omega)
    cosine = np.cos(omega)
    coeff = 2 * cosine
    q0, q1, q2 = 0, 0, 0
    for sample in samples:
        q0 = coeff * q1 - q2 + sample
        q2 = q1
        q1 = q0
    magnitude = np.sqrt(q1 ** 2 + q2 ** 2 - q1 * q2 * coeff)
    return magnitude


def find_closest_frequency(frequency_magnitudes, frequency_list):
    # Finds the frequency with the highest magnitude, ensuring dominance and threshold.

    max_freq = None
    max_magnitude = 0
    for freq in frequency_list:
        if frequency_magnitudes[freq] > max_magnitude and frequency_magnitudes[freq] > MIN_MAGNITUDE:
            max_magnitude = frequency_magnitudes[freq]
            max_freq = freq

    # Ensure dominance
    if max_freq and all(
            frequency_magnitudes[f] * DOMINANCE_RATIO < max_magnitude for f in frequency_list if f != max_freq):
        return max_freq
    return None


def decode_dtmf(signal, sample_rate):
    """
    Decodes a DTMF tone from a signal using the Goertzel algorithm.

    Args:
        signal: Input signal.
        sample_rate: Sampling rate.

    Returns:
        Detected DTMF symbol or None.
    """
    # Apply band-pass filter for DTMF range
    signal = bandpass_filter(signal, 650, 1700, sample_rate)

    # Calculate magnitudes for all DTMF frequencies
    row_magnitudes = {freq: goertzel(signal, sample_rate, freq) for freq in ROW_FREQUENCIES}
    col_magnitudes = {freq: goertzel(signal, sample_rate, freq) for freq in COLUMN_FREQUENCIES}

    # Find the row and column frequencies with the highest magnitudes
    row_freq = find_closest_frequency(row_magnitudes, ROW_FREQUENCIES)
    col_freq = find_closest_frequency(col_magnitudes, COLUMN_FREQUENCIES)

    if row_freq and col_freq:
        return DTMF_FREQUENCIES.get((row_freq, col_freq))
    return None


def audio_callback(indata, frames, callback_time, status):
    # Callback function for sounddevice to process audio in real-time.
    global last_detected, confirmation_count, route_mode, command_list, expecting_checksum, total_sum, last_tone_time

    if status:
        print(f"Audio input status: {status}")

    # Get the current time
    current_time = time.time()

    # Flatten the audio data and decode DTMF
    signal = indata[:, 0]
    sample_rate = 22500
    decoded_symbol = decode_dtmf(signal, sample_rate)

    # Non-blocking delay: Only process if enough time has passed since last tone
    if current_time - last_tone_time >= TONE_DELAY:
        last_tone_time = current_time  # Update the last tone time

        if route_mode:
            if not expecting_checksum and decoded_symbol and decoded_symbol.isdigit():
                command_list.append(decoded_symbol)
                total_sum += int(decoded_symbol)
                print(f"Added {decoded_symbol} to route plan. Total sum: {total_sum}")
            elif decoded_symbol == '*':
                print("Checksum expected next.")
                expecting_checksum = True
            elif expecting_checksum == True and decoded_symbol and decoded_symbol.isdigit():
                received_checksum = int(decoded_symbol)
                if validate_checksum(received_checksum):
                    print("Route plan checksum is valid. Starting execution.")
                    route_mode = False
                    total_sum = 0
                    execute_route()
                else:
                    print("Route plan checksum is invalid. Discarding route.")
                    command_list = []
                    route_mode = False
                expecting_checksum = False
        else:
            # Real-time remote control
            if decoded_symbol in ('1', '2', '3', '4', '5', '6', '7', '8', '9'):
                publish_movement_from_command(decoded_symbol)
            elif decoded_symbol == '#':
                print("Switching to route plan mode.")
                route_mode = True
                total_sum = 0

        # Debouncing mechanism
        if decoded_symbol == last_detected:
            confirmation_count += 1
            if confirmation_count >= REQUIRED_CONFIRMATIONS:
                print(f"Detected DTMF Tone: {decoded_symbol}")
                confirmation_count = 0  # Reset for the next symbol
        else:
            last_detected = decoded_symbol
            confirmation_count = 1
        


def main():
    # Main function to start the DTMF decoder.

    print("Starting DTMF Decoder with Goertzel Algorithm and Sensitivity Adjustments. Press Ctrl+C to stop.")
    sample_rate = 22500  # Sample rate for audio input

    with sd.InputStream(callback=audio_callback, samplerate=sample_rate, channels=1, blocksize=4096):
        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("\nStopping DTMF Decoder.")


if __name__ == "__main__":
    main()