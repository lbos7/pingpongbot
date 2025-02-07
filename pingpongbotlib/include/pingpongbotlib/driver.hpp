#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "pingpongbotlib/i2c.hpp"

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

        public:
            Driver();
            void setSpeeds(float speeds[3]);
            int8_t getSpeeds();
            void resetEncoderPulses();
            int32_t getEncoderPulses();

    };
}

#endif