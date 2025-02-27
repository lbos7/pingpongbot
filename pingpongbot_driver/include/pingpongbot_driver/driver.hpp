#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <stddef.h>
#include <iostream>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>

#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

namespace pingpongbot_driver {

    class Driver {

        private:
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
            int currentWheel1Duty;
            int currentWheel2Duty;
            int currentWheel3Duty;
            int file;
            uint8_t motorType = 0;
            uint8_t motorPolarity = 1;
            int wheel1PWM = 16;
            int wheel1INA = 20;
            int wheel1INB = 21; 
            int wheel2PWM = 13;
            int wheel2INA = 19;
            int wheel2INB = 26;
            int wheel3PWM = 12;
            int wheel3INA = 1;
            int wheel3INB = 7;
            int pwmFreq = 1000;
            int pwmRange = 100;
            int pwmPeriod = 1000;
            std::atomic<int> wheel1DutyCycle;
            std::atomic<bool> wheel1ThreadRunning;
            std::thread wheel1PWMThread;
            std::array<int32_t, 3> getEncoderPulses();
            void setup();

        public:
            Driver();
            ~Driver();
            void setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds);
            void resetEncoderPulses();
            void zeroSpeeds();
            pingpongbot_msgs::msg::WheelAngles getWheelAngles();
            std::array<int8_t, 3> getSpeeds();
            void Driver::pwmThread();
    };
}

#endif
