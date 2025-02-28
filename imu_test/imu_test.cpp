#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <cmath>

#define ISM330DHCX_ADDRESS 0x6A
#define WHO_AM_I_REG 0x0F
#define OUTX_L_XL 0x28  // Accelerometer X data register
#define OUTX_L_G 0x22   // Gyroscope X data register
#define CTRL1_XL 0x10
#define CTRL2_G 0x11

// Sensitivity values based on ±2g and ±250dps full scale settings
// const float ACCEL_SCALE = 0.000598; // ±2g (in m/s² per LSB)
const float ACCEL_SCALE = 0.001197; // ±4g (in m/s² per LSB)
const float GYRO_SCALE = 0.0175;   // ±500 dps (in dps per LSB)

int initI2C() {
    int fd = wiringPiI2CSetup(ISM330DHCX_ADDRESS);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C connection." << std::endl;
        exit(1);
    }
    return fd;
}

void checkDevice(int fd) {
    uint8_t who_am_i = wiringPiI2CReadReg8(fd, WHO_AM_I_REG);
    std::cout << "WHO_AM_I: " << std::hex << (int)who_am_i << std::dec << std::endl;
    if (who_am_i != 0x6b) {
        std::cerr << "Device not found!" << std::endl;
        exit(1);
    } else {
        std::cout << "ISM330DHCX device connected successfully." << std::endl;
    }
    // int reg_value_a = wiringPiI2CReadReg8(fd, 0x10); // Read current CTRL1_XL value
    // int reg_value_g = wiringPiI2CReadReg8(fd, 0x11); // Read current CTRL1_XL value
    // int scale_bits_a = (reg_value_a >> 2) & 0x03;
    // int scale_bits_g = (reg_value_g >> 2) & 0x03;
    // printf("Accel Scale: 0x%X\n", scale_bits_a);
    // printf("Gyro Scale: 0x%X\n", scale_bits_g);
    // std::cout << "Accel Scale: " << reg_value_a << std::endl;
    // std::cout << "Gyro Scale: " << reg_value_g << std::endl;
    wiringPiI2CWriteReg8(fd, CTRL1_XL, 0b01101010);
    wiringPiI2CWriteReg8(fd, CTRL2_G, 0b01100100);
}

// Helper function to read 16-bit data (little-endian)
int16_t read16Bit(int fd, int reg) {
    uint8_t low = wiringPiI2CReadReg8(fd, reg);
    uint8_t high = wiringPiI2CReadReg8(fd, reg + 1);
    return (int16_t)((high << 8) | low);  // Combine high and low bytes
}

void readAccelerometerData(int fd) {
    int16_t rawX = read16Bit(fd, OUTX_L_XL);
    int16_t rawY = read16Bit(fd, OUTX_L_XL + 2);
    int16_t rawZ = read16Bit(fd, OUTX_L_XL + 4);

    // Convert raw values to m/s² using scale factor
    float accelX = rawX * ACCEL_SCALE;
    float accelY = rawY * ACCEL_SCALE;
    float accelZ = rawZ * ACCEL_SCALE;

    std::cout << "Accel (m/s²): X=" << accelX << " Y=" << accelY << " Z=" << accelZ << std::endl;
}

void readGyroscopeData(int fd) {
    int16_t rawX = read16Bit(fd, OUTX_L_G);
    int16_t rawY = read16Bit(fd, OUTX_L_G + 2);
    int16_t rawZ = read16Bit(fd, OUTX_L_G + 4);

    // Convert raw values to dps using scale factor
    float gyroX = rawX * GYRO_SCALE;
    float gyroY = rawY * GYRO_SCALE;
    float gyroZ = rawZ * GYRO_SCALE;

    std::cout << "Gyro (dps): X=" << gyroX << " Y=" << gyroY << " Z=" << gyroZ << std::endl;
}

int main() {
    int fd = initI2C();
    checkDevice(fd);

    while (true) {
        readAccelerometerData(fd);
        readGyroscopeData(fd);
        usleep(100000); // Delay 100ms
    }

    return 0;
}
