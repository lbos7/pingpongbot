#include "pingpongbot_driver/driver.hpp"

namespace pingpongbot_driver {

    Driver::Driver() {
        setup();
    }

    Driver::~Driver() {
        zeroSpeeds();
        this->wheel1ThreadRunning.store(false);
        if (wheel1PWMThread.joinable()) {
            wheel1PWMThread.join();  // Wait for thread to finish
        }
        digitalWrite(this->wheel1INA, LOW);
        digitalWrite(this->wheel1INB, LOW);
        digitalWrite(this->wheel2INA, LOW);
        digitalWrite(this->wheel2INB, LOW);
        digitalWrite(this->wheel3INA, LOW);
        digitalWrite(this->wheel3INB, LOW);
    }

    std::array<int8_t, 3> Driver::getSpeeds() {
        std::array<int8_t, 3> speeds;
        for (int i = 0; i < 3; i++) {
            speeds[i] = wiringPiI2CReadReg8(file, motorFixedPWMAddr + i);
        }
        return speeds;
    }

    void Driver::setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds) {
        this->currentWheel1Duty = std::clamp((int) (std::abs(speeds.u1 * this->motor1RadPS2PWM)), 0, 100);
        this->currentWheel2Duty = std::clamp((int) (std::abs(speeds.u2 * this->motor1RadPS2PWM)), 0, 100);
        this->currentWheel3Duty = std::clamp((int) (std::abs(speeds.u3 * this->motor1RadPS2PWM)), 0, 100);

        this->wheel1DutyCycle.store(this->currentWheel1Duty);
        pwmWrite(this->wheel2PWM, this->currentWheel2Duty);
        pwmWrite(this->wheel3PWM, this->currentWheel3Duty);
        
        if (speeds.u1 > 0) {
            digitalWrite(this->wheel1INA, LOW);
            digitalWrite(this->wheel1INB, HIGH);
        } else if (speeds.u1 < 0) {
            digitalWrite(this->wheel1INA, HIGH);
            digitalWrite(this->wheel1INB, LOW);
        }

        if (speeds.u2 > 0) {
            digitalWrite(this->wheel2INA, LOW);
            digitalWrite(this->wheel2INB, HIGH);
        } else if (speeds.u2 < 0) {
            digitalWrite(this->wheel2INA, HIGH);
            digitalWrite(this->wheel2INB, LOW);
        }

        if (speeds.u3 > 0) {
            digitalWrite(this->wheel3INA, LOW);
            digitalWrite(this->wheel3INB, HIGH);
        } else if (speeds.u3 < 0) {
            digitalWrite(this->wheel3INA, HIGH);
            digitalWrite(this->wheel3INB, LOW);
        }
    }



    std::array<int32_t, 3> Driver::getEncoderPulses() {
        std::array<int32_t, 3> counts = {0, 0, 0};

        // Read 16 bytes of encoder data from the I2C device
        uint8_t encoderData[16];
        for (int i = 0; i < 16; i++) {
            encoderData[i] = wiringPiI2CReadReg8(this->file, this->motorEncoderTotalAddr + i);
        }

        // Process the data to extract counts
        for (int i = 0; i < 3; i++) {
            // Combine 4 bytes to form a 32-bit integer (little-endian format)
            int32_t count = encoderData[i * 4] | (encoderData[i * 4 + 1] << 8) |
                            (encoderData[i * 4 + 2] << 16) | (encoderData[i * 4 + 3] << 24);

            // Apply the sign inversion for the second encoder if needed
            if (i == 1) {
                counts[i] = count * -1;
            } else {
                counts[i] = count;
            }
        }

        return counts;
    }


    void Driver::setup() {
        this->file = wiringPiI2CSetup(this->motorEncoderTotalAddr);
        if (this->file == -1) {
            // Handle I2C initialization failure
            std::cerr << "Failed to initialize I2C device" << std::endl;
        }
        if (file < 0) {
            std::cerr << "Failed to initialize I2C device" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        wiringPiI2CWriteReg8(file, motorTypeAddr, motorType);
        wiringPiI2CWriteReg8(file, motorEncoderPolarityAddr, motorPolarity);
        if (wiringPiSetupGpio() == -1) {
            std::cerr << "WiringPi initialization failed!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        pinMode(this->wheel1PWM, OUTPUT);
        pinMode(this->wheel1INA, OUTPUT);
        pinMode(this->wheel1INB, OUTPUT);
        pinMode(this->wheel2PWM, PWM_OUTPUT);
        pinMode(this->wheel2INA, OUTPUT);
        pinMode(this->wheel2INB, OUTPUT);
        pinMode(this->wheel3PWM, PWM_OUTPUT);
        pinMode(this->wheel3INA, OUTPUT);
        pinMode(this->wheel3INB, OUTPUT);

        pwmSetMode(PWM_MODE_MS); // Mark-Space mode for precise frequency
        pwmSetRange(this->pwmRange);  // Set range for duty cycle (0-100 for percentage control)
        this->wheel1PWMThread = std::thread(&Driver::pwmThread, this);

    }

    pingpongbot_msgs::msg::WheelAngles Driver::getWheelAngles() {
        std::array<int32_t, 3> counts = getEncoderPulses();
        pingpongbot_msgs::msg::WheelAngles adjustedWheelAngles;
        adjustedWheelAngles.theta1 = counts[0] * radPerCount;
        adjustedWheelAngles.theta2 = counts[2] * radPerCount;
        adjustedWheelAngles.theta3 = counts[1] * radPerCount;
        return adjustedWheelAngles;
    }

    void Driver::resetEncoderPulses() {
        bool done = false;
        bool countsReset[3] = {false, false, false};
        std::array<int32_t, 3> counts;
    
        // Create a buffer of zeros to send to the encoder to reset the pulses
        uint8_t encoderZero[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
        while (!done) {
            // Write the encoder reset data (16 bytes of zeros)
            for (int i = 0; i < 16; i++) {
                wiringPiI2CWriteReg8(this->file, this->motorEncoderTotalAddr + i, encoderZero[i]);
            }
    
            // Read the encoder pulses to check if reset was successful
            counts = this->getEncoderPulses();
    
            // Check if all encoder counts are zero (indicating reset)
            for (int i = 0; i < 3; i++) {
                countsReset[i] = counts[i] == 0;
            }
    
            // If all encoders are reset, exit the loop
            done = countsReset[0] && countsReset[1] && countsReset[2];
        }
    }

    void Driver::zeroSpeeds() {
        pingpongbot_msgs::msg::WheelSpeeds speeds;
        speeds.u1 = 0;
        speeds.u2 = 0;
        speeds.u3 = 0;
        this->setSpeeds(speeds);
    }
    

    void Driver::pwmThread() {

        while (this->wheel1ThreadRunning.load()) {
            int highTime = (this->pwmPeriod * this->wheel1DutyCycle.load()) / 100;  // Load current duty cycle
            int lowTime = this->pwmPeriod - highTime;  // Calculate LOW time

            digitalWrite(this->wheel1PWM, HIGH);  // Set the pin HIGH
            delayMicroseconds(highTime);      // Wait for HIGH time
            digitalWrite(this->wheel1PWM, LOW);   // Set the pin LOW
            delayMicroseconds(lowTime);      // Wait for LOW time
        }

        std::cout << "PWM thread stopped." << std::endl;
    }
}
