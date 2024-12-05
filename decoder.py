import numpy as np
import sounddevice as sd
from scipy.signal import butter, lfilter

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
REQUIRED_CONFIRMATIONS = 1  # Number of confirmations to avoid false positives

# Globals for debouncing
last_detected = None
confirmation_count = 0

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
    magnitude = np.sqrt(q1**2 + q2**2 - q1 * q2 * coeff)
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
    if max_freq and all(frequency_magnitudes[f] * DOMINANCE_RATIO < max_magnitude for f in frequency_list if f != max_freq):
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

def audio_callback(indata, frames, time, status):
    
    # Callback function for sounddevice to process audio in real-time.
    
    global last_detected, confirmation_count
    if status:
        print(f"Audio input status: {status}")

    # Flatten the audio data and decode DTMF
    signal = indata[:, 0]
    sample_rate = 44100
    decoded_symbol = decode_dtmf(signal, sample_rate)

    # Debouncing mechanism to avoid false positives
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
    sample_rate = 44100  # Sample rate for audio input

    with sd.InputStream(callback=audio_callback, samplerate=sample_rate, channels=1, blocksize=4096):
        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("\nStopping DTMF Decoder.")

if __name__ == "__main__":
    main()
