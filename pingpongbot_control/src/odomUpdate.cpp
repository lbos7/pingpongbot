#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2/convert.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"
#include "pingpongbot_control/kinematics.hpp"

class OdometryUpdate: public rclcpp::Node {

    public:
        OdometryUpdate() : Node("odom_update") {
            this->declare_parameter("d", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("r", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("odom_id", "odom");
            this->declare_parameter("base_id", "base_footprint");

            d = this->get_parameter("d").as_double();
            r = this->get_parameter("r").as_double();
            odom_id = this->get_parameter("odom_id").as_string();
            base_id = this->get_parameter("base_id").as_string();

            omni_drive = pingpongbot_control::OmniDrive(d, r);

            odom_trans.header.frame_id = odom_id;
            odom_trans.child_frame_id = base_id;

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

            wheel_speeds_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelSpeeds>("wheel_speeds", 10);

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&OdometryUpdate::jointStateCallback, this, std::placeholders::_1));

            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&OdometryUpdate::cmdVelCallback, this, std::placeholders::_1));

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:
        void jointStateCallback(const sensor_msgs::msg::JointState & msg) {
            auto current_time = this->get_clock()->now();
            if (!first_joints_cb) {
                auto relative_trans = omni_drive.odomUpdate(msg);
                auto new_trans = relative_trans * prev_trans;

                auto dt = (current_time - prev_time).seconds();
                odom_msg.header.stamp = current_time;
                odom_msg.pose.pose.position.x = new_trans.getOrigin().x();
                odom_msg.pose.pose.position.y = new_trans.getOrigin().y();

                odom_msg.pose.pose.orientation.x = new_trans.getRotation().normalized().getX();
                odom_msg.pose.pose.orientation.y = new_trans.getRotation().normalized().getY();
                odom_msg.pose.pose.orientation.z = new_trans.getRotation().normalized().getZ();
                odom_msg.pose.pose.orientation.w = new_trans.getRotation().normalized().getW();
                
                odom_msg.twist.twist.linear.x =
                    (new_trans.getOrigin().x() - prev_trans.getOrigin().x()) / dt;
                odom_msg.twist.twist.linear.y =
                    (new_trans.getOrigin().y() - prev_trans.getOrigin().y()) / dt;

                auto new_trans_rotation = new_trans.getRotation().normalized();
                auto prev_trans_rotation = prev_trans.getRotation().normalized();

                double new_yaw, new_pitch, new_roll;
                tf2::getEulerYPR(new_trans_rotation, new_yaw, new_pitch, new_roll);

                double prev_yaw, prev_pitch, prev_roll;
                tf2::getEulerYPR(prev_trans_rotation, prev_yaw, prev_pitch, prev_roll);

                odom_msg.twist.twist.angular.z = (new_yaw - prev_yaw) / dt;

                odom_trans.header.stamp = current_time;
                odom_trans.transform.translation.x = new_trans.getOrigin().x();
                odom_trans.transform.translation.y = new_trans.getOrigin().y();
                odom_trans.transform.rotation = tf2::toMsg(new_trans.getRotation().normalized());

                tf_broadcaster_->sendTransform(odom_trans);

                prev_trans = new_trans;
            } else {
                first_joints_cb = false;

            }
            prev_time = current_time;
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist & msg) {
            auto speeds = omni_drive.twist2WheelSpeeds(msg);
            wheel_speeds_pub_->publish(speeds);
        }

        double d, r;
        std::string odom_id;
        std::string base_id;
        bool first_joints_cb = true;
        pingpongbot_control::OmniDrive omni_drive;
        tf2::Transform prev_trans;
        sensor_msgs::msg::JointState currentJointState;
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::TransformStamped odom_trans;
        rclcpp::Time prev_time;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryUpdate>());
  rclcpp::shutdown();
  return 0;
}