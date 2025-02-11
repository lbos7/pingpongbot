#include "pingpongbotlib/kinematics.hpp"

namespace pingpongbotlib {

    OmniDrive::OmniDrive(const double d, const double r) {
        this->d = d;
        this->r = r;
    }

    std::array<double, 3> twist2WheelSpeeds(geometry_msgs::msg::Twist twist) {
        auto wz = twist.angular.z;
        auto vx = twist.linear.x;
        auto vy = twist.linear.y;

        std::array<double, 3> wheelSpeeds;

        wheelSpeeds[0] = (double) (1/this->r)*(-this->d*wz + vx);
        wheelSpeeds[1] = (double) (1/this->r)*(-this->d*wz - .5*vx - (pow(3, .5)/2)*vy);
        wheelSpeeds[2] = (double) (1/this->r)*(-this->d*wz - .5*vx + (pow(3, .5)/2)*vy);
        
        return wheelSpeeds;
    }

    tf2::Transform odomUpdate(std::array<double, 3> newWheelPositions) {
        double u1 = newWheelPositions[0] - this->wheelPositons[0];
        double u2 = newWheelPositions[1] - this->wheelPositons[1];
        double u3 = newWheelPositions[2] - this->wheelPositons[2];

        auto dYaw = this->r*(-u1/(3*this->d) - u2/(3*this->d) - u3/(3*this->d));
        auto dx = this->r*((2/3)*u1 - u2/3 - u3/3);
        auto dy = this->r*(-u2/(2*(pow(3, .5)/2)) + u3/(2*(pow(3, .5)/2)))

        auto translation = tf2::Vector3(dx, dy, 0.0);
        auto rotation = tf2::Quaternion{};
        rotation.setRPY(0.0, 0.0, dYaw);

        this->wheelPositions = newWheelPositions;

        return tf2::Transform(rotation, translation);
    }
}