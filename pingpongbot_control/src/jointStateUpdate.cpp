#include <chrono>

#include "sensor_msgs/msg/joint_state.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class JointStateUpdate : public rclcpp::Node {

    public:
        JointStateUpdate() : Node("joint_state_update") {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&JointStateUpdate::timer_callback, this));

            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&PingPongBotDriver::wheelSpeedsCallback, this, std::placeholders::_1));

            wheel_angles_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelAngles>(
                "wheel_angles", 10, std::bind(&PingPongBotDriver::wheelAnglesCallback, this, std::placeholders::_1));
        }

    private:
        void timer_callback() {

        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds msg) {

        }
        
        void wheelAnglesCallback(const pingpongbot_msgs::msg::WheelAngles msg) {
            
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::Angles>::SharedPtr wheel_angles_sub_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateUpdate>());
  rclcpp::shutdown();
  return 0;
}