#include "pingpongbot_driver/driver.hpp"

namespace pingpongbot_driver {

    Driver::Driver() {
        setup();
    }

    Driver::~Driver() {
        close(this->file);
    }

    std::array<int8_t, 3> Driver::getSpeeds() {
        int8_t speedInts[4] = {0, 0, 0, 0};
        std::array<int8_t, 3> speeds;
        i2cRead(this->file, this->motorFixedPWMAddr, (uint8_t*)speedInts, 4);
        for (int i = 0; i < 3; i++) {
            speeds[i] = speedInts[i];
        }
        return speeds;
    }

    void Driver::setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds) {
        bool done = false;
        bool speedsSet[3] = {false, false, false};
        int8_t speedInts[4] = {0, 0, 0, 0};
        std::array<int8_t, 3> speedsCheck;
        // Note - Motor order is CCW due to driver, but wheel order is CW due to calcs
        speedInts[0] = (int8_t) (speeds.u1 * this->motor1RadPS2PWM);
        speedInts[1] = (int8_t) (speeds.u3 * this->motor2RadPS2PWM * -1);
        speedInts[2] = (int8_t) (speeds.u2 * this->motor3RadPS2PWM);
        for (int i = 0; i < 3; i++) {
            speedInts[i] = (int8_t) std::clamp((int)speedInts[i], -100, 100);
        }
        while (!done) {
            i2cWrite(this->file, this->motorFixedPWMAddr, (uint8_t*)speedInts, 4);
            speedsCheck = this->getSpeeds();
            for (int i = 0; i < 3; i++) {
                speedsSet[i] = speedInts[i] == speedsCheck[i];
            }
            done = speedsSet[0] && speedsSet[1] && speedsSet[2];
        }
    }

    std::array<int32_t, 3> Driver::getEncoderPulses() {
        uint8_t encoderData[16];
        std::array<int32_t, 3> counts = {0, 0, 0};
        i2cRead(this->file, this->motorEncoderTotalAddr, encoderData, 16);
        for (int i = 0; i < 3; i++) {
            int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                            (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);
            if (i == 1) {
                counts[i] = count * -1;
            } else {
                counts[i] = count;
            }
        }
        return counts;
    }

    void Driver::setup() {
        i2cWrite(this->file, this->motorTypeAddr, &(this->motorType), 1);
        i2cWrite(this->file, this->motorEncoderPolarityAddr, &(this->motorPolarity), 1);
    }

    pingpongbot_msgs::msg::WheelAngles Driver::getWheelAngles() {
        std::array<int32_t, 3> counts = this->getEncoderPulses();
        std::array<double, 3> wheelAngles = {0.0, 0.0, 0.0};
        pingpongbot_msgs::msg::WheelAngles adjustedWheelAngles;
        for (int i = 0; i < 3; i++) {
            wheelAngles[i] = ((double) counts[i]) * this->radPerCount;
        }
        // Note - Motor order is CCW due to driver, but wheel order is CW due to calcs
        adjustedWheelAngles.theta1 = wheelAngles[0];
        adjustedWheelAngles.theta2 = wheelAngles[2];
        adjustedWheelAngles.theta3 = wheelAngles[1];
        return adjustedWheelAngles;
    }

    void Driver::resetEncoderPulses() {
        bool done = false;
        bool countsReset[3] = {false, false, false};
        std::array<int32_t, 3> counts;
        uint8_t encoderZero[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        while (!done) {
            i2cWrite(this->file, this->motorEncoderTotalAddr, (uint8_t*)encoderZero, 16);
            counts = this->getEncoderPulses();
            for (int i = 0; i < 3; i++) {
                countsReset[i] = counts[i] == 0;
            }
            done = countsReset[0] && countsReset[1] && countsReset[2];
        }
    }

    int openI2CBus(const char* I2C_DEVICE, uint8_t MOTOR_DRIVER_ADDR) {
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
}