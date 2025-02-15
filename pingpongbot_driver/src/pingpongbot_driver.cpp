#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "pingpongbot_driver/driver.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class PingPongBotDriver : public rclcpp::Node {
    
    public:
        PingPongBotDriver() : Node("pingpongbot_driver") {

            driver = std::make_shared<pingpongbot_driver::Driver>();

            // Check if driver_ is correctly initialized
            if (!driver) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize the Driver object.");
            }

            driver->resetEncoderPulses();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&PingPongBotDriver::timerCallback, this));

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&PingPongBotDriver::wheelSpeedsCallback, this, std::placeholders::_1));

            heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
                "heartbeat", 10, std::bind(&PingPongBotDriver::heartbeatCallback, this, std::placeholders::_1));
        }

        ~PingPongBotDriver() {
            RCLCPP_INFO(this->get_logger(), "Shutting down PingPongBotDriver...");
            driver.reset(); // Explicitly reset the driver before node shutdown
        }
    

    private:
        void timerCallback() {
            auto msg = driver->getWheelAngles();
            wheel_angles_pub_->publish(msg);
            driver->setSpeeds(speeds);
            auto current_time = this->get_clock()->now();
            if ((current_time - last_heartbeat_time_).seconds() > 0.5) {
                RCLCPP_WARN(this->get_logger(), "Lost connection! Stopping wheels.");
                pingpongbot_msgs::msg::WheelSpeeds zero;
                zero.u1 = 0;
                zero.u2 = 0;
                zero.u3 = 0;
                driver->setSpeeds(zero);
            }
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds & msg) {
            speeds = msg;
        }

        void heartbeatCallback(const std_msgs::msg::Empty & msg) {
            (void) msg;
            last_heartbeat_time_ = this->get_clock()->now();
        }

        // pingpongbot_driver::Driver driver;
        std::shared_ptr<pingpongbot_driver::Driver> driver;
        pingpongbot_msgs::msg::WheelSpeeds speeds;
        rclcpp::Time last_heartbeat_time_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongBotDriver>());
    rclcpp::shutdown();
    return 0;
}