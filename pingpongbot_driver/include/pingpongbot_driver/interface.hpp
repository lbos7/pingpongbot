#ifndef INTERFACE_HPP
#define INTERFACE_HPP

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
#include "pingpongbot_msgs/msg/imu.hpp"

namespace pingpongbot_driver {

    class Interface {

        private:
            uint8_t motorDriverAddr = 0x34;
            uint8_t motorTypeAddr = 0x14;
            uint8_t motorEncoderPolarityAddr = 0x15;
            uint8_t motorFixedPWMAddr = 0x1F;
            uint8_t motorFixedSpeedAddr = 0x33;
            uint8_t motorEncoderTotalAddr = 0x3C;
            uint8_t imuAddr = 0x6A;
            uint8_t xAccelLowAddr = 0x28;
            uint8_t xAccelHighAddr = 0x29;
            uint8_t yAccelLowAddr = 0x2A;
            uint8_t yAccelHighAddr = 0x2B;
            uint8_t zAccelLowAddr = 0x2C;
            uint8_t zAccelHighAddr = 0x2D;
            uint8_t xGyroLowAddr = 0x22;
            uint8_t xGyroHighAddr = 0x23;
            uint8_t yGyroLowAddr = 0x24;
            uint8_t yGyroHighAddr = 0x25;
            uint8_t zGyroLowAddr = 0x26;
            uint8_t zGyroHighAddr = 0x27;
            uint8_t accelSetupAddr = 0x10;
            uint8_t gyroSetupAddr = 0x11;
            uint8_t pwmDriverAddr = 0x40;
            uint8_t wheel1OnLAddr = 0x22;
            uint8_t wheel1OnHAddr = 0x23;
            uint8_t wheel1OffLAddr = 0x24;
            uint8_t wheel1OffHAddr = 0x25;
            uint8_t wheel2OnLAddr = 0x12;
            uint8_t wheel2OnHAddr = 0x13;
            uint8_t wheel2OffLAddr = 0x14;
            uint8_t wheel2OffHAddr = 0x15;
            uint8_t wheel3OnLAddr = 0x32;
            uint8_t wheel3OnHAddr = 0x33;
            uint8_t wheel3OffLAddr = 0x34;
            uint8_t wheel3OffHAddr = 0x35;
            uint8_t modeAddr = 0x00;
            uint8_t prescalerAddr = 0xFE;
            double motor1RadPS2PWM = .60122 * .63;
            double motor2RadPS2PWM = .61565 * .63;
            double motor3RadPS2PWM = .61747 * .63;
            double radPerCount = .01570769;
            double accelScale = 0.001197;
            double gyroScale = 0.0175 * (2*M_PI)/360;
            double xaOffset = 0;
            double yaOffset = 0;
            double zaOffset = 0;
            double xgOffset = 0;
            double ygOffset = 0;
            double zgOffset = 0;
            int currentWheel1Duty;
            int currentWheel2Duty;
            int currentWheel3Duty;
            int fd;
            int fi;
            int fp;
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
            pingpongbot_msgs::msg::IMU calculateIMUOffsets();
            void setPWM(int wheelNum, int dutyCycle);
            void setup();

        public:
            Interface();
            ~Interface();
            void setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds);
            void resetEncoderPulses();
            void zeroSpeeds();
            pingpongbot_msgs::msg::WheelAngles getWheelAngles();
            std::array<int8_t, 3> getSpeeds();
            pingpongbot_msgs::msg::IMU getIMUData();
            void setIMUOffsets(pingpongbot_msgs::msg::IMU data);
    };
}

#endif
