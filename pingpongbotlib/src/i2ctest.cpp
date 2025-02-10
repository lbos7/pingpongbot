#include "pingpongbotlib/driver.hpp"
#include <iostream>
#include <unistd.h>
#include <cstdint>
#include <chrono>
#include <vector>
#include <numeric>
#include <fstream>

int main() {

    bool done = false;
    float speed[3] = {0, 0, 0};
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> times = {};

    pingpongbotlib::Driver driver;
    driver.resetEncoderPulses();

    while (!done) {
        speed[0] = 32.5;
        speed[1] = 32.5;
        speed[2] = 32.5;

        driver.setSpeeds(speed);

        auto now = std::chrono::high_resolution_clock::now();  // Current time
        double elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed >= 1) {
            done = !done;
        }

        times.push_back(elapsed);
        
    }
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    driver.setSpeeds(speed);
    std::array<float, 3> wheelAngles = driver.getWheelAngles();
    for (int i = 0; i < 3; i++) {
        std::cout << wheelAngles[i] << " ";
    }
    std::cout << std::endl;
    return 0;
}
