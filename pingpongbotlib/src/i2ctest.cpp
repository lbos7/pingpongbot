#include "pingpongbotlib/i2c.hpp"
#include <iostream>
#include <unistd.h>
#include <cstdint>
#include <chrono>


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
    uint8_t motorPolarity = 0;
    pingpongbotlib::i2cWrite(file, motorEncoderPolarityAddr, &motorPolarity, 1);

    bool done = false;
    int8_t speed[4] = {0, 0, 0, 0};
    auto start = std::chrono::high_resolution_clock::now();

    while (!done) {
        speed[0] = -50;
        speed[1] = 50;
        speed[2] = -50;
        pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);

        auto now = std::chrono::high_resolution_clock::now();  // Current time
        double elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed >= 1) {
            done = !done;
        }
        
    }
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);
    pingpongbotlib::i2cWrite(file, motorFixedPWMAddr, (uint8_t*)speed, 4);

    // Read encoder values
    uint8_t encoderData[16] = {0};
    pingpongbotlib::i2cRead(file, motorEncoderTotalAddr, encoderData, 16);

    std::cout << "Encoder Readings:\n";
    for (int i = 0; i < 4; i++) {
        int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                        (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);
        std::cout << "Motor " << (i + 1) << ": " << count << " pulses\n";
    }

    close(file);
    return 0;
}
