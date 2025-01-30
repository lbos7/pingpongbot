#include "i2c.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

int openI2CBus(char* I2C_DEVICE, uint8_t MOTOR_DRIVER_ADDR) {
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

bool i2cWrite(int file, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }
    return write(file, buffer, len + 1) == len + 1;
}

bool i2cRead(int file, uint8_t reg, uint8_t *data, size_t len) {
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Error setting register address\n";
        return false;
    }
    return read(file, data, len) == len;
}
