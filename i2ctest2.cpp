#include "i2c.hpp"
#include <cstdint>
#include <chrono>
#include <unistd.h>

int main() {

    const uint8_t MOTOR_DRIVER_ADDR = 0x34;
    const uint8_t MOTOR_TYPE_ADDR = 0x14;
    const uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
    const uint8_t MOTOR_FIXED_PWM_ADDR = 0x1F;
    const uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;
    const uint8_t MOTOR_ENCODER_TOTAL_ADDR = 0x3c;
    char* I2C_DEVICE = "/dev/i2c-1";
    int file;
    uint8_t motorType = 0;
    uint8_t encoderPolarity = 0;
    uint8_t encoderData[16] = {0};
    
    file = openI2CBus(I2C_DEVICE, MOTOR_DRIVER_ADDR);

    i2cRead(file, MOTOR_ENCODER_TOTAL_ADDR, encoderData, 16);

    close(file);
}