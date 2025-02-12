#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <algorithm>
#include <array>
#include <unistd.h>

#include "pingpongbotlib/i2c.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

namespace pingpongbotlib {

    class Driver {

        private:
            const char* i2cDevice = "/dev/i2c-1";
            const uint8_t motorDriverAddr = 0x34;
            const uint8_t motorTypeAddr = 0x14;
            const uint8_t motorEncoderPolarityAddr = 0x15;
            const uint8_t motorFixedPWMAddr = 0x1F;
            const uint8_t motorFixedSpeedAddr = 0x33;
            const uint8_t motorEncoderTotalAddr = 0x3C;
            const double motor1RadPS2PWM = .60122;
            const double motor2RadPS2PWM = .61565;
            const double motor3RadPS2PWM = .61747;
            const double radPerCount = .01570769;
            const int file = pingpongbotlib::openI2CBus(i2cDevice, motorDriverAddr);
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