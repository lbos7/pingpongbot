#include "pingpongbot_control/kinematics.hpp"

namespace pingpongbot_control {

    OmniDrive::OmniDrive(const double d, const double r) {
        this->d = d;
        this->r = r;
    }

    pingpongbot_msgs::msg::WheelSpeeds OmniDrive::twist2WheelSpeeds(geometry_msgs::msg::Twist twist) {
        auto wz = twist.angular.z;
        auto vx = twist.linear.x;
        auto vy = twist.linear.y;

        pingpongbot_msgs::msg::WheelSpeeds wheelSpeeds;

        wheelSpeeds.u1 = (double) (1/this->r)*(-this->d*wz + vx);
        wheelSpeeds.u2 = (double) (1/this->r)*(-this->d*wz - .5*vx - (pow(3, .5)/2)*vy);
        wheelSpeeds.u2 = (double) (1/this->r)*(-this->d*wz - .5*vx + (pow(3, .5)/2)*vy);
        
        return wheelSpeeds;
    }

    tf2::Transform OmniDrive::odomUpdate(sensor_msgs::msg::JointState newState) {

        pingpongbot_msgs::msg::WheelAngles newWheelPositions;
        newWheelPositions.theta1 = newState.position[0];
        newWheelPositions.theta2 = newState.position[1];
        newWheelPositions.theta3 = newState.position[2];

        double dtheta1 = newWheelPositions.theta1 - this->wheelPositions.theta1;
        double dtheta2 = newWheelPositions.theta2 - this->wheelPositions.theta2;
        double dtheta3 = newWheelPositions.theta3 - this->wheelPositions.theta3;

        auto dYaw = this->r*(-dtheta1/(3*this->d) - dtheta2/(3*this->d) - dtheta3/(3*this->d));
        auto dx = this->r*((2/3)*dtheta1 - dtheta2/3 - dtheta3/3);
        auto dy = this->r*(-dtheta2/(2*(pow(3, .5)/2)) + dtheta3/(2*(pow(3, .5)/2)));

        auto translation = tf2::Vector3(dx, dy, 0.0);
        auto rotation = tf2::Quaternion{};
        rotation.setRPY(0.0, 0.0, dYaw);

        this->wheelPositions = newWheelPositions;

        return tf2::Transform(rotation, translation);
    }
}