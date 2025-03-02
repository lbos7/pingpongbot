#include "pingpongbot_driver/interface.hpp"

namespace pingpongbot_driver {

    Interface::Interface() {
        setup();
    }

    Interface::~Interface() {
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

    std::array<int8_t, 3> Interface::getSpeeds() {
        std::array<int8_t, 3> speeds;
        for (int i = 0; i < 3; i++) {
            speeds[i] = wiringPiI2CReadReg8(this->fd, this->motorFixedPWMAddr + i);
        }
        return speeds;
    }

    void Interface::setSpeeds(pingpongbot_msgs::msg::WheelSpeeds speeds) {
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



    std::array<int32_t, 3> Interface::getEncoderPulses() {
        std::array<int32_t, 3> counts = {0, 0, 0};

        // Read 16 bytes of encoder data from the I2C device
        uint8_t encoderData[16];
        for (int i = 0; i < 16; i++) {
            encoderData[i] = wiringPiI2CReadReg8(this->fd, this->motorEncoderTotalAddr + i);
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


    void Interface::setup() {
        this->fd = wiringPiI2CSetup(this->motorDriverAddr);
        this->fi = wiringPiI2CSetup(this->imuAddr);
        if (this->fd == -1) {
            std::cerr << "Failed to initialize Driver Board" << std::endl;
            exit(1);
        }
        if (this->fi == -1) {
            std::cerr << "Failed to initialize IMU" << std::endl;
            exit(1);
        }
        if (wiringPiSetupGpio() == -1) {
            std::cerr << "WiringPi initialization failed!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        wiringPiI2CWriteReg8(fd, this->accelSetupAddr, 0b01101010);
        wiringPiI2CWriteReg8(fd, this->gyroSetupAddr, 0b01100100);
        wiringPiI2CWriteReg8(this->fd, this->motorTypeAddr, this->motorType);
        wiringPiI2CWriteReg8(this->fd, this->motorEncoderPolarityAddr, this->motorPolarity);

        pingpongbot_msgs::msg::IMU offsets = this->calculateIMUOffsets();
        this->setIMUOffsets(offsets);

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
        this->wheel1PWMThread = std::thread(&Interface::pwmThread, this);

    }

    pingpongbot_msgs::msg::WheelAngles Interface::getWheelAngles() {
        std::array<int32_t, 3> counts = getEncoderPulses();
        pingpongbot_msgs::msg::WheelAngles adjustedWheelAngles;
        adjustedWheelAngles.theta1 = counts[0] * radPerCount;
        adjustedWheelAngles.theta2 = counts[2] * radPerCount;
        adjustedWheelAngles.theta3 = counts[1] * radPerCount;
        return adjustedWheelAngles;
    }

    void Interface::resetEncoderPulses() {
        bool done = false;
        bool countsReset[3] = {false, false, false};
        std::array<int32_t, 3> counts;
    
        // Create a buffer of zeros to send to the encoder to reset the pulses
        uint8_t encoderZero[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
        while (!done) {
            // Write the encoder reset data (16 bytes of zeros)
            for (int i = 0; i < 16; i++) {
                wiringPiI2CWriteReg8(this->fd, this->motorEncoderTotalAddr + i, encoderZero[i]);
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

    void Interface::zeroSpeeds() {
        pingpongbot_msgs::msg::WheelSpeeds speeds;
        speeds.u1 = 0;
        speeds.u2 = 0;
        speeds.u3 = 0;
        this->setSpeeds(speeds);
    }

    pingpongbot_msgs::msg::IMU Interface::getIMUData() {

        uint8_t xaLow = wiringPiI2CReadReg8(this->fi, this->xAccelLowAddr);
        uint8_t xaHigh = wiringPiI2CReadReg8(this->fi, this->xAccelHighAddr);
        uint8_t yaLow = wiringPiI2CReadReg8(this->fi, this->yAccelLowAddr);
        uint8_t yaHigh = wiringPiI2CReadReg8(this->fi, this->yAccelHighAddr);
        uint8_t zaLow = wiringPiI2CReadReg8(this->fi, this->zAccelLowAddr);
        uint8_t zaHigh = wiringPiI2CReadReg8(this->fi, this->zAccelHighAddr);
        uint8_t xgLow = wiringPiI2CReadReg8(this->fi, this->xGyroLowAddr);
        uint8_t xgHigh = wiringPiI2CReadReg8(this->fi, this->xGyroHighAddr);
        uint8_t ygLow = wiringPiI2CReadReg8(this->fi, this->yGyroLowAddr);
        uint8_t ygHigh = wiringPiI2CReadReg8(this->fi, this->yGyroHighAddr);
        uint8_t zgLow = wiringPiI2CReadReg8(this->fi, this->zGyroLowAddr);
        uint8_t zgHigh = wiringPiI2CReadReg8(this->fi, this->zGyroHighAddr);

        int16_t xaRaw = (int16_t)((xaHigh << 8) | xaLow);
        int16_t yaRaw = (int16_t)((yaHigh << 8) | yaLow);
        int16_t zaRaw = (int16_t)((zaHigh << 8) | zaLow);
        int16_t xgRaw = (int16_t)((xgHigh << 8) | xgLow);
        int16_t ygRaw = (int16_t)((ygHigh << 8) | ygLow);
        int16_t zgRaw = (int16_t)((zgHigh << 8) | zgLow);

        pingpongbot_msgs::msg::IMU msg;
        msg.xa = (xaRaw * this->accelScale) - xaOffset;
        msg.ya = (yaRaw * this->accelScale) - yaOffset;
        msg.za = (zaRaw * this->accelScale) - zaOffset;
        msg.xg = (xgRaw * this->gyroScale) - xgOffset;
        msg.yg = (ygRaw * this->gyroScale) - ygOffset;
        msg.zg = (zgRaw * this->gyroScale) - zgOffset;
        
        return msg;
    }

    pingpongbot_msgs::msg::IMU Interface::calculateIMUOffsets() {

        pingpongbot_msgs::msg::IMU offsets;
        pingpongbot_msgs::msg::IMU newData;

        for (int i = 0; i < 100; i++) {
            newData = this->getIMUData();
            offsets.xa += newData.xa;
            offsets.ya += newData.ya;
            offsets.za += newData.za;
            offsets.xg += newData.xg;
            offsets.yg += newData.yg;
            offsets.zg += newData.zg;
        }

        offsets.xa /= 100;
        offsets.ya /= 100;
        offsets.za /= 100;
        offsets.xg /= 100;
        offsets.yg /= 100;
        offsets.zg /= 100;

        return offsets;
    }

    void Interface::setIMUOffsets(pingpongbot_msgs::msg::IMU data) {
        this->xaOffset = data.xa;
        this->yaOffset = data.ya;
        this->zaOffset = data.za - 9.81;
        this->xgOffset = data.xg;
        this->ygOffset = data.yg;
        this->zgOffset = data.zg;
    }
    

    void Interface::pwmThread() {

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
