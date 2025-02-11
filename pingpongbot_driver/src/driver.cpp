#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pingpongbotlib/driver.hpp"

class Driver : public rclcpp::Node {
    public:
        Driver() : Node("driver") {
            
        }

    private:
        
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Driver>());
    rclcpp::shutdown();
    return 0;
}