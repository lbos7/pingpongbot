#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class JointStateUpdate : public rclcpp::Node {

    public:
        JointStateUpdate() : Node("joint_state_update") {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&JointStateUpdate::timerCallback, this));

            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&JointStateUpdate::wheelSpeedsCallback, this, std::placeholders::_1));

            wheel_angles_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelAngles>(
                "wheel_angles", 10, std::bind(&JointStateUpdate::wheelAnglesCallback, this, std::placeholders::_1));
            
        }

    private:
        void timerCallback() {
            sensor_msgs::msg::JointState msg;
            msg.header.frame_id = "base_link";
            msg.header.stamp = this->get_clock()->now();
            msg.name = {"wheel1_joint", "wheel2_joint", "wheel3_joint"};
            msg.velocity = {currentSpeeds.u1, currentSpeeds.u2, currentSpeeds.u3};
            msg.position = {currentAngles.theta1, currentAngles.theta2, currentAngles.theta3};
            this->joint_state_pub_->publish(msg);
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds & msg) {
            currentSpeeds = msg;
        }
        
        void wheelAnglesCallback(const pingpongbot_msgs::msg::WheelAngles & msg) {
            currentAngles = msg;
        }

        pingpongbot_msgs::msg::WheelSpeeds currentSpeeds;
        pingpongbot_msgs::msg::WheelAngles currentAngles;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateUpdate>());
  rclcpp::shutdown();
  return 0;
}