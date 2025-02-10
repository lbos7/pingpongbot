#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define I2C_DEVICE "/dev/i2c-1"  // I2C bus on Raspberry Pi
#define MOTOR_DRIVER_ADDR 0x34   // I2C address of the motor driver

// Motor Driver Register Addresses
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_PWM_ADDR 0x1F
#define MOTOR_FIXED_SPEED_ADDR 0x33
#define MOTOR_ENCODER_TOTAL_ADDR 0x3c

// Open I2C communication
int openI2CBus() {
    int file = open(I2C_DEVICE, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, MOTOR_DRIVER_ADDR) < 0) {
        std::cerr << "Failed to connect to motor driver\n";
        return -1;
    }
    return file;
}

// Write data to a register
bool i2cWrite(int file, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }
    return write(file, buffer, len + 1) == len + 1;
}

// Read data from a register
bool i2cRead(int file, uint8_t reg, uint8_t *data, size_t len) {
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Error setting register address\n";
        return false;
    }
    return read(file, data, len) == len;
}

int main() {
    int file = openI2CBus();
    if (file < 0) return 1;

    // Set motor type (TT motor)
    uint8_t motorType = 3;  // MOTOR_TYPE_TT
    i2cWrite(file, MOTOR_TYPE_ADDR, &motorType, 1);

    // Set motor encoder polarity
    uint8_t motorPolarity = 1;
    i2cWrite(file, MOTOR_ENCODER_POLARITY_ADDR, &motorPolarity, 1);

    // Set motor speed (forward)
    int8_t speed[4] = {0, 0, 0, 0};  // All motors at 50 speed
    i2cWrite(file, MOTOR_FIXED_SPEED_ADDR, (uint8_t*)speed, 4);

    sleep(2);

    // Stop motors
    int8_t stop[4] = {0, 0, 0, 0};
    i2cWrite(file, MOTOR_FIXED_SPEED_ADDR, (uint8_t*)stop, 4);

    // Read encoder values
    uint8_t encoderData[16] = {0};
    i2cRead(file, MOTOR_ENCODER_TOTAL_ADDR, encoderData, 16);

    std::cout << "Encoder Readings:\n";
    for (int i = 0; i < 4; i++) {
        int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                        (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);
        std::cout << "Motor " << (i + 1) << ": " << count << " pulses\n";
    }

    close(file);
    return 0;
}