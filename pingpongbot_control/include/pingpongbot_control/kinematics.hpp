#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

namespace pingpongbot_control {

    class OmniDrive {

        private:
            double d;
            double r;

        public:
            OmniDrive(){}
            OmniDrive(const double d, const double r);
            pingpongbot_msgs::msg::WheelSpeeds twist2WheelSpeeds(geometry_msgs::msg::Twist twist);
            tf2::Transform odomUpdate(sensor_msgs::msg::JointState newState, sensor_msgs::msg::JointState oldState);
    };

}

#endif