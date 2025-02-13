#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"
#include "pingpongbot_control/kinematics.hpp"

class Controller : public rclcpp::Node {

    public:
        Controller() : Node("controller") {
            this->declare_parameter("d", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("r", rclcpp::ParameterType::PARAMETER_DOUBLE);

            d = this->get_parameter("d").as_double();
            r = this->get_parameter("r").as_double();

            omni_drive = pingpongbot_control::OmniDrive(d, r);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Controller::timerCallback, this));

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&Controller::jointStateCallback, this, std::placeholders::_1));
        }

    private:
        void timerCallback() {
            
        }

        void jointStateCallback(const sensor_msgs::msg::JointState & msg) {
            currentJointState = msg;
        }

        double d, r;
        sensor_msgs::msg::JointState currentJointState;
        pingpongbot_control::OmniDrive omni_drive;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}