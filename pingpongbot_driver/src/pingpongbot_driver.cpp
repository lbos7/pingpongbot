#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pingpongbotlib/driver.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class PingPongBotDriver : public rclcpp::Node {
    public:
        PingPongBotDriverr() : Node("pingpongbot_driver") {

            driver = pingpongbotlib::Driver();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&PingPongBotDriver::timer_callback, this);
            )

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&PingPongBotDriver::wheelSpeedsCallback, this, std::placeholders::_1));
        }

    private:
        void timer_callback() {
            auto msg = driver.getWheelAngles();
            wheel_angles_pub_->publish(msg);
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds msg) {
            driver.setSpeeds(msg);
        }

        rclcpp::rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongBotDriver>());
    rclcpp::shutdown();
    return 0;
}