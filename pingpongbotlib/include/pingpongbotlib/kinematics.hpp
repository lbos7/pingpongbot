#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

namespace pingpongbotlib {

    class OmniDrive {

        private:
            double d;
            double r;
            pingpongbot_msgs::msg::WheelAngles wheelPositions;

        public:
            OmniDrive(){}
            OmniDrive(const double d, const double r);
            pingpongbot_msgs::msg::WheelSpeeds twist2WheelSpeeds(geometry_msgs::msg::Twist twist);
            tf2::Transform odomUpdate(pingpongbot_msgs::msg::WheelAngles newWheelPositions);
    }

}

#endif