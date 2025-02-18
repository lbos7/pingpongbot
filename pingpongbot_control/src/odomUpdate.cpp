#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2/convert.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
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
            this->declare_parameter("alpha", 0.5);

            d = this->get_parameter("d").as_double();
            r = this->get_parameter("r").as_double();
            odom_id = this->get_parameter("odom_id").as_string();
            base_id = this->get_parameter("base_id").as_string();
            alpha = this->get_parameter("alpha").as_double();

            rclcpp::QoS qos(10);
            qos.reliable();
            qos.transient_local();

            omni_drive = pingpongbot_control::OmniDrive(d, r);

            odom_trans.header.frame_id = odom_id;
            odom_trans.child_frame_id = base_id;

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&OdometryUpdate::timerCallback, this));

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

            wheel_speeds_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelSpeeds>("wheel_speeds", 10);

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&OdometryUpdate::jointStateCallback, this, std::placeholders::_1));

            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&OdometryUpdate::cmdVelCallback, this, std::placeholders::_1));

            reached_goal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "reached_goal", 10, std::bind(&OdometryUpdate::reachedGoalCallback, this, std::placeholders::_1));

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this, qos);

            reset_encoder_cli_ = this->create_client<std_srvs::srv::Empty>("reset_encoder");

            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&OdometryUpdate::onParamChange, this, std::placeholders::_1));
        }

        ~OdometryUpdate() {
            RCLCPP_INFO(this->get_logger(), "Shutting down OdomUpdate...");
            pingpongbot_msgs::msg::WheelSpeeds zero = pingpongbot_msgs::msg::WheelSpeeds();
            zero.u1 = 0;
            zero.u2 = 0;
            zero.u3 = 0;
            wheel_speeds_pub_->publish(zero);
            sendResetRequest();
        }

    private:
        void timerCallback() {
            tf_broadcaster_->sendTransform(odom_trans);
        }
        
        void jointStateCallback(const sensor_msgs::msg::JointState & msg) {
            auto current_time = this->get_clock()->now();
            if (!first_joints_cb) {
                auto relative_trans = omni_drive.odomUpdate(msg);
                auto new_trans = relative_trans * prev_trans;

                auto dt = (current_time - prev_time).seconds();

                // if (std::sqrt(std::pow(new_trans.getOrigin().getX() - prev_trans.getOrigin().getX(), 2)
                //     + std::pow(new_trans.getOrigin().getY() - prev_trans.getOrigin().getY(), 2)) >= .2) {
                //         new_trans = prev_trans;
                // }

                odom_msg.header.stamp = current_time;
                odom_msg.header.frame_id = odom_id;
                odom_msg.child_frame_id = base_id;
                odom_msg.pose.pose.position.x = alpha * new_trans.getOrigin().x() + (1 - alpha) * prev_trans.getOrigin().x();
                odom_msg.pose.pose.position.y = alpha * new_trans.getOrigin().y() + (1 - alpha) * prev_trans.getOrigin().y();
                odom_msg.pose.pose.orientation.x = alpha * new_trans.getRotation().normalized().getX() 
                + (1 - alpha) * prev_trans.getRotation().normalized().getX();
                odom_msg.pose.pose.orientation.y = alpha * new_trans.getRotation().normalized().getY() 
                + (1 - alpha) * prev_trans.getRotation().normalized().getY();
                odom_msg.pose.pose.orientation.z = alpha * new_trans.getRotation().normalized().getZ()
                + (1 - alpha) * prev_trans.getRotation().normalized().getZ();
                odom_msg.pose.pose.orientation.w = alpha * new_trans.getRotation().normalized().getW()
                + (1 - alpha) * prev_trans.getRotation().normalized().getW();

                odom_pub_->publish(odom_msg);

                // RCLCPP_INFO(get_logger(), "Location of 'base_link' in 'odom': [%.2f, %.2f]",
                //         odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
                
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

                // tf_broadcaster_->sendTransform(odom_trans);
                prev_trans = new_trans;

            } else {
                first_joints_cb = false;
                // odom_trans = geometry_msgs::msg::TransformStamped();
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = odom_id;
                odom_trans.child_frame_id = base_id;
                geometry_msgs::msg::Transform trans = odom_trans.transform;
                tf2::fromMsg(trans, prev_trans);
                // tf_broadcaster_->sendTransform(odom_trans);

            }
            prev_time = current_time;
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist & msg) {
            auto speeds = omni_drive.twist2WheelSpeeds(msg);
            // RCLCPP_INFO(
            //     rclcpp::get_logger("wheel_speeds"),
            //     "Speeds:[1: %.2f 2: %.2f, 3: %.2f]",
            //     speeds.u1, speeds.u2, speeds.u3
            // );
            if (!reached_goal) {
                wheel_speeds_pub_->publish(speeds);
            } else {
                pingpongbot_msgs::msg::WheelSpeeds zero;
                zero.u1 = 0;
                zero.u2 = 0;
                zero.u3 = 0;
                wheel_speeds_pub_->publish(zero);
            }
        }

        void reachedGoalCallback(const std_msgs::msg::Bool & msg) {
            reached_goal = msg.data;
            if (reached_goal && !prev_check) {
                sendResetRequest();
            }
            prev_check = reached_goal;
        }

        void sendResetRequest() {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();

            auto future_result = reset_encoder_cli_->async_send_request(request,
                std::bind(&OdometryUpdate::response_callback, this, std::placeholders::_1));
        }

        void response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response received");
        }

        rcl_interfaces::msg::SetParametersResult onParamChange(
            const std::vector<rclcpp::Parameter> & params) {
            for (const auto &param : params) {
                if (param.get_name() == "alpha") {
                    alpha = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated alpha: %.3f", alpha);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

        double d, r;
        std::string odom_id;
        std::string base_id;
        bool first_joints_cb = true;
        bool first_timer_cb = true;
        pingpongbot_control::OmniDrive omni_drive;
        tf2::Transform prev_trans;
        sensor_msgs::msg::JointState currentJointState;
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::TransformStamped odom_trans;
        bool reached_goal = false;
        bool prev_check = false;
        rclcpp::Time prev_time;
        double alpha;
        double dist_check;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_goal_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_encoder_cli_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryUpdate>());
  rclcpp::shutdown();
  return 0;
}