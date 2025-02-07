#include "pingpongbotlib/i2c.hpp"
#include <iostream>
#include <unistd.h>
#include <cstdint>
#include <chrono>
#include <vector>
#include <numeric>
#include <fstream>

void saveIntVectorToCSV(const std::vector<int>& data, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file!\n";
        return;
    }

    for (size_t i = 0; i < data.size(); ++i) {
        file << data[i];
        if (i < data.size() - 1) {
            file << ",";  // Separate values with commas
        }
    }

    file << "\n";  // Newline at the end
    file.close();
    std::cout << "Data saved to " << filename << "\n";
}

void saveDoubleVectorToCSV(const std::vector<double>& data, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file!\n";
        return;
    }

    for (size_t i = 0; i < data.size(); ++i) {
        file << data[i];
        if (i < data.size() - 1) {
            file << ",";  // Separate values with commas
        }
    }

    file << "\n";  // Newline at the end
    file.close();
    std::cout << "Data saved to " << filename << "\n";
}



int main() {
    const char* i2cDevice = "/dev/i2c-1";
    const uint8_t motorDriverAddr = 0x34;
    const uint8_t motorTypeAddr = 0x14;
    const uint8_t motorEncoderPolarityAddr = 0x15;
    const uint8_t motorFixedPWMAddr = 0x1F;
    const uint8_t motorFixedSpeedAddr = 0x33;
    const uint8_t motorEncoderTotalAddr = 0x3C;
    int file = pingpongbotlib::openI2CBus(i2cDevice, motorDriverAddr);

    // Set motor type (TT motor)
    uint8_t motorType = 0;  // MOTOR_TYPE_TT
    pingpongbotlib::i2cWrite(file, motorTypeAddr, &motorType, 1);

    // Set motor encoder polarity
    uint8_t motorPolarity = 1;
    pingpongbotlib::i2cWrite(file, motorEncoderPolarityAddr, &motorPolarity, 1);

    bool done = false;
    int8_t speed[4] = {0, 0, 0, 0};
    auto start = std::chrono::high_resolution_clock::now();
    // auto prev = start;
    std::vector<double> times = {};
    std::vector<int32_t> counts1 = {};
    std::vector<int32_t> counts2 = {};
    std::vector<int32_t> counts3 = {};

    uint8_t encoderData[16];

    uint8_t encoderZero[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    pingpongbotlib::i2cWrite(file, motorEncoderTotalAddr, (uint8_t*)encoderZero, 16);
    pingpongbotlib::i2cWrite(file, motorEncoderTotalAddr, (uint8_t*)encoderZero, 16);
    pingpongbotlib::i2cWrite(file, motorEncoderTotalAddr, (uint8_t*)encoderZero, 16);
    pingpongbotlib::i2cWrite(file, motorEncoderTotalAddr, (uint8_t*)encoderZero, 16);

    while (!done) {
        speed[0] = -10;
        speed[1] = 10;
        speed[2] = -10;
        pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);

        auto now = std::chrono::high_resolution_clock::now();  // Current time
        double elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed >= 2) {
            done = !done;
        }

        times.push_back(elapsed);

        pingpongbotlib::i2cRead(file, motorEncoderTotalAddr, encoderData, 16);

        for (int i = 0; i < 4; i++) {
            int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                            (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);
            if (i == 0) {
                counts1.push_back(count);
            } else if (i == 1) {
                counts2.push_back(count);
            } else if (i == 2) {
                counts3.push_back(count);
            }
        }
        
    }
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);
    pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);

    // Read encoder values
    // uint8_t encoderData[16];
    pingpongbotlib::i2cRead(file, motorEncoderTotalAddr, encoderData, 16);

    std::cout << "Encoder Readings:\n";
    for (int i = 0; i < 4; i++) {
        int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                        (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);
        std::cout << "Motor " << (i + 1) << ": " << count << " pulses\n";
    }

    saveDoubleVectorToCSV(times, "times_10.csv");
    saveIntVectorToCSV(counts1, "counts1_10.csv");
    saveIntVectorToCSV(counts2, "counts2_10.csv");
    saveIntVectorToCSV(counts3, "counts3_10.csv");

    close(file);
    return 0;
}
