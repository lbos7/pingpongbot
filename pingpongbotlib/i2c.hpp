#ifndef I2C_H
#define I2C_H
#include <cstdint>
#include <stddef.h>

int openI2CBus(const char* I2C_DEVICE, uint8_t MOTOR_DRIVER_ADDR);
bool i2cWrite(int file, uint8_t reg, uint8_t *data, size_t len);
bool i2cRead(int file, uint8_t reg, uint8_t *data, size_t len);

#endif