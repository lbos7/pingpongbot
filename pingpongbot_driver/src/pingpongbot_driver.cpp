#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
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

            reset_encoder_srv_ = this->create_service<std_srvs::srv::Empty>(
                "reset_encoder", std::bind(&PingPongBotDriver::resetEncoder, this, std::placeholders::_1, std::placeholders::_2));
        }

        ~PingPongBotDriver() {
            RCLCPP_INFO(this->get_logger(), "Shutting down PingPongBotDriver...");
            driver.reset(); // Explicitly reset the driver before node shutdown
        }
    

    private:
        void timerCallback() {
            auto msg = driver->getWheelAngles();
            wheel_angles_pub_->publish(msg);
            // pingpongbot_msgs::msg::WheelSpeeds speeds;
            // speeds.u1 = 60;
            // speeds.u2 = 60;
            // speeds.u3 = 60;
            driver->setSpeeds(speeds);
            auto current_time = this->clock_.now();
            if ((current_time - last_heartbeat_time).seconds() > 0.175) {
                // RCLCPP_WARN(this->get_logger(), "Lost connection! Stopping wheels.");
                driver->zeroSpeeds();
                // driver->resetEncoderPulses();
            }
            static pingpongbot_msgs::msg::WheelAngles prev_angles;  // Store previous wheel angles

            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "Wheel Angles: θ1: %.6f, θ2: %.6f, θ3: %.6f", 
            //             msg.theta1, msg.theta2, msg.theta3);

            // // Compute and log delta values (change in wheel angles)
            // double delta_theta1 = msg.theta1 - prev_angles.theta1;
            // double delta_theta2 = msg.theta2 - prev_angles.theta2;
            // double delta_theta3 = msg.theta3 - prev_angles.theta3;

            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "ΔTheta: dθ1: %.6f, dθ2: %.6f, dθ3: %.6f",
            //             delta_theta1, delta_theta2, delta_theta3);

            // std::array<int8_t, 3> speedInts = driver->getSpeeds();

            // Update previous angles
            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "Motor Speeds: 1: %.6f, 2: %.6f, 3: %.6f", 
            //             speedInts[0], speedInts[1], speedInts[2]);
            prev_angles = msg;
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds & msg) {
            speeds = msg;
        }

        void heartbeatCallback(const std_msgs::msg::Empty & msg) {
            (void) msg;
            last_heartbeat_time = this->clock_.now();
        }

        void resetEncoder(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                this->driver->resetEncoderPulses();
        }

        std::shared_ptr<pingpongbot_driver::Driver> driver;
        pingpongbot_msgs::msg::WheelSpeeds speeds;
        rclcpp::Time last_heartbeat_time;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock clock_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_encoder_srv_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongBotDriver>());
    rclcpp::shutdown();
    return 0;
}