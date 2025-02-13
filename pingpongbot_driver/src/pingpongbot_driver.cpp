#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pingpongbot_driver/driver.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class PingPongBotDriver : public rclcpp::Node {
    
    public:
        PingPongBotDriver() : Node("pingpongbot_driver") {

            driver = pingpongbot_driver::Driver();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&PingPongBotDriver::timerCallback, this));

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&PingPongBotDriver::wheelSpeedsCallback, this, std::placeholders::_1));
        }

    private:
        void timerCallback() {
            auto msg = driver.getWheelAngles();
            wheel_angles_pub_->publish(msg);
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds & msg) {
            driver.setSpeeds(msg);
        }

        pingpongbot_driver::Driver driver;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongBotDriver>());
    rclcpp::shutdown();
    return 0;
}