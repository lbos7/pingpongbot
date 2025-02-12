#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <algorithm>
#include <array>
#include <stddef.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

namespace pingpongbot_driver {

    int openI2CBus(const char* I2C_DEVICE, uint8_t MOTOR_DRIVER_ADDR);

    bool i2cWrite(int file, uint8_t reg, uint8_t *data, size_t len);

    bool i2cRead(int file, uint8_t reg, uint8_t *data, size_t len);

    class Driver {

        private:
            const char* i2cDevice = "/dev/i2c-1";
            uint8_t motorDriverAddr = 0x34;
            uint8_t motorTypeAddr = 0x14;
            uint8_t motorEncoderPolarityAddr = 0x15;
            uint8_t motorFixedPWMAddr = 0x1F;
            uint8_t motorFixedSpeedAddr = 0x33;
            uint8_t motorEncoderTotalAddr = 0x3C;
            double motor1RadPS2PWM = .60122;
            double motor2RadPS2PWM = .61565;
            double motor3RadPS2PWM = .61747;
            double radPerCount = .01570769;
            int file = openI2CBus(i2cDevice, motorDriverAddr);
            uint8_t motorType = 0;
            uint8_t motorPolarity = 1;
            std::array<int8_t, 3> getSpeeds();
            std::array<int32_t, 3> getEncoderPulses();
            void setup();

        public:
            Driver();
            ~Driver();
            void setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds);
            void resetEncoderPulses();
            pingpongbot_msgs::msg::WheelAngles getWheelAngles();

    };

}

#endif