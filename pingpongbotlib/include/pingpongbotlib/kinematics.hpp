#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <array>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/msg/twist.hpp"

namespace pingpongbotlib {

    class OmniDrive {

        private:
            double d;
            double r;
            std::array<double, 3> wheelPositions;

        public:
            OmniDrive(){}
            OmniDrive(const double d, const double r);
            std::array<double, 3> twist2WheelSpeeds(geometry_msgs::msg::Twist twist);
            tf2::Transform odomUpdate(std::array<double, 3> wheelDisplacements);
    }

}

#endif