#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

int main () {
    const char *device = "/dev/i2c-1";
    int file;
    const int address = 0x34;

    if ((file = open(device, O_RDWR)) < 0) {
        std::cerr << "Failed to open the I2C bus" << std::endl;
        return 1;
    } else {
        std::cout << "Opened I2C bus" << std::endl;
    }
}