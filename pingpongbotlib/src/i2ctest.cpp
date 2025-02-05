#include "pingpongbotlib/i2c.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

// #define I2C_DEVICE "/dev/i2c-1"  // I2C bus on Raspberry Pi
// const char* i2cDevice = "/dev/i2c-1";
// #define MOTOR_DRIVER_ADDR 0x34   // I2C address of the motor driver

// // Motor Driver Register Addresses
// #define MOTOR_TYPE_ADDR 0x14
// #define MOTOR_ENCODER_POLARITY_ADDR 0x15
// #define MOTOR_FIXED_PWM_ADDR 0x1F
// #define MOTOR_FIXED_SPEED_ADDR 0x33
// #define MOTOR_ENCODER_TOTAL_ADDR 0x3C


int main() {
    const char* i2cDevice = "/dev/i2c-1";
    const uint8_t motorDriverAddr = 0x34;
    const uint8_t motorTypeAddr = 0x14;
    const uint8_t motorEncoderPolarityAddr = 0x15;
    // const uint8_t motorFixedPWMAddr = 0x1F;
    const uint8_t motorFixedSpeedAddr = 0x33;
    const uint8_t motorEncoderTotalAddr = 0x3C;
    int file = pingpongbotlib::openI2CBus(i2cDevice, motorDriverAddr);

    // Set motor type (TT motor)
    uint8_t motorType = 0;  // MOTOR_TYPE_TT
    pingpongbotlib::i2cWrite(file, motorTypeAddr, &motorType, 1);

    // Set motor encoder polarity
    uint8_t motorPolarity = 1;
    pingpongbotlib::i2cWrite(file, motorEncoderPolarityAddr, &motorPolarity, 1);

    // Set motor speed (forward)
    int8_t speed[4] = {0, 0, 0, 0};  // All motors at 50 speed
    pingpongbotlib::i2cWrite(file, motorFixedSpeedAddr, (uint8_t*)speed, 4);

    sleep(2);

    // Stop motors
    int8_t stop[4] = {0, 0, 0, 0};
    pingpongbotlib::i2cWrite(file, motorFixedSpeedAddr, (uint8_t*)stop, 4);

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