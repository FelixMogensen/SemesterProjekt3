 #include <SFML/Audio.hpp>
#include <SFML/Window.hpp>
#include <cmath>
#include <iostream>
#include <map>

const int SAMPLE_RATE = 44100;
const double PI = 3.14159265358979323846;
const int AMPLITUDE = 30000;

// Map DTMF frequencies
std::map<sf::Keyboard::Key, std::pair<int, int>> dtmfFrequencies = {
    {sf::Keyboard::Num1, {697, 1209}}, // 1
    {sf::Keyboard::Num2, {697, 1336}}, // 2
    {sf::Keyboard::Num3, {697, 1477}}, // 3
    {sf::Keyboard::Num4, {770, 1209}}, // 4
    {sf::Keyboard::Num5, {770, 1336}}, // 5
    {sf::Keyboard::Num6, {770, 1477}}, // 6
    {sf::Keyboard::Num7, {852, 1209}}, // 7
    {sf::Keyboard::Num8, {852, 1336}}, // 8
    {sf::Keyboard::Num9, {852, 1477}}, // 9
    {sf::Keyboard::O, {941, 1209}},     // * (mapped to O)
    {sf::Keyboard::Num0, {941, 1336}},  // 0
    {sf::Keyboard::P, {941, 1477}},      // # (mapped to P)
    {sf::Keyboard::A, {697, 1633}},      // # (mapped to P)
    {sf::Keyboard::B, {770, 1633}}      // # (mapped to P)
};

// Generate a sine wave for the specified frequency
void generateSineWave(std::vector<sf::Int16>& samples, double frequency, int durationInSeconds) {
    int sampleCount = SAMPLE_RATE * durationInSeconds;
    samples.resize(sampleCount);
    for (int i = 0; i < sampleCount; ++i) {
        samples[i] = static_cast<sf::Int16>(AMPLITUDE * sin(2 * PI * frequency * i / SAMPLE_RATE));
    }
}

// Generate DTMF tone by combining two frequencies
void generateDTMFTone(sf::Keyboard::Key key, std::vector<sf::Int16>& samples, int durationInSeconds) {
    if (dtmfFrequencies.find(key) == dtmfFrequencies.end()) return;

    auto frequencies = dtmfFrequencies[key];
    int sampleCount = SAMPLE_RATE * durationInSeconds;
    samples.resize(sampleCount);

    for (int i = 0; i < sampleCount; ++i) {
        double sample = 0.5 * (sin(2 * PI * frequencies.first * i / SAMPLE_RATE) +
                               sin(2 * PI * frequencies.second * i / SAMPLE_RATE));
        samples[i] = static_cast<sf::Int16>(AMPLITUDE * sample);
    }
}

int main() {
    sf::SoundBuffer buffer;
    sf::Sound sound;

    std::cout << "Press and hold keys 1-9, 0, O (*), or P (#) to play corresponding DTMF tones. Press 'Q' to quit." << std::endl;

    while (true) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) break;

        // Check each DTMF key
        for (const auto& pair : dtmfFrequencies) {
            sf::Keyboard::Key key = pair.first;

            // Check if the corresponding key is pressed
            if (sf::Keyboard::isKeyPressed(key)) {
                std::vector<sf::Int16> samples;
                generateDTMFTone(key, samples, 1); // Generate a 1-second tone

                // Load samples into buffer and play
                if (buffer.loadFromSamples(samples.data(), samples.size(), 1, SAMPLE_RATE)) {
                    sound.setBuffer(buffer);
                    sound.setLoop(true); // Loop the sound while the key is held down
                    sound.play();

                    // Wait until the key is released
                    while (sf::Keyboard::isKeyPressed(key)) {
                        // Keep playing until the key is released
                    }

                    sound.stop(); // Stop the sound when the key is released
                }
            }
        }
    }

    std::cout << "Program ended." << std::endl;
    return 0;
}


// g++ Tester.cpp -o dtmf_realtime -lsfml-audio -lsfml-system -lsfml-window