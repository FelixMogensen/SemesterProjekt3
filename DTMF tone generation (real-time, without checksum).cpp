#include <SFML/Audio.hpp>
#include <SFML/Window.hpp>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

const int SAMPLE_RATE = 44100; // Standard sample rate
const double PI = 3.14159265358979323846; // Pi constant for calculations
const int AMPLITUDE = 30000; // Amplitude
const double DURATION = 0.4; // Tone duration in seconds

// Mapping for DTMF tones
std::map<char, std::pair<int, int>> dtmfFrequencies = {
    {'F', {852, 1336}}, {'B', {697, 1336}}, // Forward (F) and Back (B)
    {'L', {770, 1209}}, {'R', {770, 1477}}, // Left (L) and Right (R)
    {'0', {941, 1336}}, {'1', {697, 1209}}, {'2', {697, 1336}}, {'3', {697, 1477}},
    {'4', {770, 1209}}, {'5', {770, 1336}}, {'6', {770, 1477}}, {'7', {852, 1209}}, {'8', {852, 1336}}, {'9', {852, 1477}}  
};

// DTMF tone generation with debugging
void generateDTMFTone(char c, std::vector<sf::Int16>& samples, double durationInSeconds) {
    if (dtmfFrequencies.find(c) == dtmfFrequencies.end()) {
        std::cerr << "Error: Character '" << c << "' not found in DTMF frequencies map.\n";
        return;
    }

    auto frequencies = dtmfFrequencies[c];
    double sampleCount = SAMPLE_RATE * durationInSeconds;
    samples.resize(sampleCount);

    for (int i = 0; i < sampleCount; ++i) {
        double sample = 0.5 * (sin(2 * PI * frequencies.first * i / SAMPLE_RATE) +
                               sin(2 * PI * frequencies.second * i / SAMPLE_RATE));
        samples[i] = static_cast<sf::Int16>(AMPLITUDE * sample);
    }

    // Debugging, Log sample size
    if (samples.empty()) {
        std::cerr << "Error: Samples array is empty for character '" << c << "'.\n";
    } else {
        std::cout << "Generated " << samples.size() << " samples for character '" << c << "'.\n";
    }
}

// Transmit message function with validation
void transmitMessage(const std::string& message, sf::SoundBuffer& buffer, sf::Sound& sound) {
    for (char c : message) {
        std::vector<sf::Int16> samples;
        generateDTMFTone(c, samples, DURATION);

        if (samples.empty()) {
            std::cerr << "Error: No samples generated for character '" << c << "'. Skipping...\n";
            continue;
        }

        if (buffer.loadFromSamples(samples.data(), samples.size(), 1, SAMPLE_RATE)) {
            sound.setBuffer(buffer);
            sound.play();
            sf::sleep(sf::seconds(DURATION)); // Ensure proper timing
        } else {
            std::cerr << "Failed to load sound buffer for character '" << c << "'.\n";
        }
    }
}

int main() {
    sf::SoundBuffer buffer;
    sf::Sound sound;

    std::cout << "Use arrow keys to control the robot. Press 'Q' to quit." << std::endl;

    while (true) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) break;

        char command = '\0';

        // Check for arrow key presses
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) command = 'F';
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) command = 'B';
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) command = 'L';
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) command = 'R';
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num5)) command = '5'; // Added key press for '5'

        if (command != '\0') {
            std::string message(1, command); // Directly use the command as the message
            std::cout << "Sending command: " << message << std::endl;

            transmitMessage(message, buffer, sound);

            // Wait until the key is released to prevent repeated transmissions
            while (sf::Keyboard::isKeyPressed(sf::Keyboard::Up) ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::Down) ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::Left) ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::Right) ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::Num5)) {
            }
        }
    }

    std::cout << "Program ended." << std::endl;
    return 0;
}

// Compile command on mac:
// g++ DTMFrealtimeNOcheck.cpp -o DTMFrealtimeNOcheck -I/opt/homebrew/opt/sfml/include -L/opt/homebrew/opt/sfml/lib -lsfml-audio -lsfml-system -lsfml-window -std=c++11
