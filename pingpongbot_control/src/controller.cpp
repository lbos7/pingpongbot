#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

class Controller : public rclcpp::Node {
    public:
        Controller() : Node("control") {
            this->declare_parameter("Kp_x", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Ki_x", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Kd_x", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Kp_y", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Ki_y", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Kd_y", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Kp_ang", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Ki_ang", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("Kd_ang", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("odom_id", "odom");
            this->declare_parameter("base_id", "base_footprint");
            this->declare_parameter("thresh_lin", 0.055);
            this->declare_parameter("thresh_ang", 0.0349066);

            Kp_x = this->get_parameter("Kp_x").as_double();
            Ki_x = this->get_parameter("Ki_x").as_double();
            Kd_x = this->get_parameter("Kd_x").as_double();
            Kp_y = this->get_parameter("Kp_y").as_double();
            Ki_y = this->get_parameter("Ki_y").as_double();
            Kd_y = this->get_parameter("Kd_y").as_double();
            Kp_ang = this->get_parameter("Kp_ang").as_double();
            Ki_ang = this->get_parameter("Ki_ang").as_double();
            Kd_ang = this->get_parameter("Kd_ang").as_double();
            odom_id = this->get_parameter("odom_id").as_string();
            base_id = this->get_parameter("base_id").as_string();
            thresh_lin = this->get_parameter("thresh_lin").as_double();
            thresh_ang = this->get_parameter("thresh_ang").as_double();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Controller::timerCallback, this));

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);

            goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "goal_pose", 10, std::bind(&Controller::goalPoseCallback, this, std::placeholders::_1));
            
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }

        ~Controller() {
            RCLCPP_INFO(this->get_logger(), "Shutting down Controller...");
            geometry_msgs::msg::Twist zero;
            cmd_vel_pub_->publish(zero);
        }
    
    private:
        void timerCallback() {
            auto current_time = this->get_clock()->now();

            if (!first_cb) {
                try {
                    t = tf_buffer_->lookupTransform(odom_id, base_id,tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(  // Use WARN instead of INFO for errors
                        this->get_logger(), 
                        "Could not transform %s to %s: %s",
                        odom_id.c_str(), base_id.c_str(), ex.what());
                    return;
                }

                auto dt = (current_time - prev_time).seconds();

                double goal_x = currentGoal.pose.position.x;
                double goal_y = currentGoal.pose.position.y;

                double goal_yaw, goal_pitch, goal_roll;
                tf2::Quaternion currentGoal_quat;
                tf2::fromMsg(currentGoal.pose.orientation, currentGoal_quat);
                tf2::getEulerYPR(currentGoal_quat, goal_yaw, goal_pitch, goal_roll);

                double current_x = t.transform.translation.x;
                double current_y = t.transform.translation.y;

                double current_yaw, current_pitch, current_roll;
                tf2::Quaternion t_quat;
                tf2::fromMsg(t.transform.rotation, t_quat);
                tf2::getEulerYPR(t_quat, current_yaw, current_pitch, current_roll);
                
                error_linx = goal_x - current_x;
                error_liny = goal_y - current_y;
                error_ang = goal_yaw - current_yaw;

                distance_error = std::sqrt(std::pow(error_linx, 2) + std::pow(error_liny, 2));

                if ((distance_error < thresh_lin) && (std::abs(error_ang) < thresh_ang)) {

                    commandedTwist.linear.x = 0;
                    commandedTwist.linear.y = 0;
                    commandedTwist.angular.z = 0;

                    cmd_vel_pub_->publish(commandedTwist);

                } else {
                    accum_error_linx += error_linx * dt;
                    accum_error_liny += error_liny * dt;
                    accum_error_ang += error_ang * dt;

                    auto vx = (Kp_x * error_linx) + (Ki_x * accum_error_linx) + (Kd_x * ((error_linx - prev_error_linx)/dt));
                    auto vy = (Kp_y * error_liny) + (Ki_y * accum_error_liny) + (Kd_y * ((error_liny - prev_error_liny)/dt));
                    auto wz = (Kp_ang * error_ang) + (Ki_ang * accum_error_ang) + (Kd_ang * ((error_ang - prev_error_ang)/dt));

                    commandedTwist.linear.x = vx;
                    commandedTwist.linear.y = vy;
                    commandedTwist.angular.z = wz;

                    RCLCPP_INFO(
                        rclcpp::get_logger("twist_logger"),
                        "Twist Message - Linear: [x: %.2f, y: %.2f, z: %.2f], Angular: [x: %.2f, y: %.2f, z: %.2f]",
                        commandedTwist.linear.x, commandedTwist.linear.y, commandedTwist.linear.z,
                        commandedTwist.angular.x, commandedTwist.angular.y, commandedTwist.angular.z
                    );

                    cmd_vel_pub_->publish(commandedTwist);
                    
                    prev_error_linx = error_linx;
                    prev_error_liny = error_liny;
                    prev_error_ang = error_ang;
                }

            } else {
                first_cb = false;
                current_state.data = false;
                shutdown_pub_->publish(current_state);
            }
            prev_time = current_time;
            std_msgs::msg::Empty empty;
            heartbeat_pub_->publish(empty);
        }

        void goalPoseCallback(const geometry_msgs::msg::PoseStamped & msg) {
            currentGoal = msg;
            RCLCPP_INFO(
                rclcpp::get_logger("goal_pose_logger"),
                "PoseStamped Message:\n"
                "Header - frame_id: %s, timestamp: %u.%u\n"
                "Position - x: %.2f, y: %.2f, z: %.2f\n"
                "Orientation - x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                currentGoal.header.frame_id.c_str(),
                currentGoal.header.stamp.sec, currentGoal.header.stamp.nanosec,
                currentGoal.pose.position.x, currentGoal.pose.position.y, currentGoal.pose.position.z,
                currentGoal.pose.orientation.x, currentGoal.pose.orientation.y,
                currentGoal.pose.orientation.z, currentGoal.pose.orientation.w
            );
        }

        double Kp_x, Ki_x, Kd_x;
        double Kp_y, Ki_y, Kd_y;
        double Kp_ang, Ki_ang, Kd_ang;
        double error_linx, prev_error_linx = 0, accum_error_linx = 0;
        double error_liny, prev_error_liny = 0, accum_error_liny = 0;
        double distance_error;
        double error_ang, prev_error_ang = 0, accum_error_ang = 0;
        std::string odom_id;
        std::string base_id;
        double thresh_lin, thresh_ang;
        geometry_msgs::msg::PoseStamped currentGoal = geometry_msgs::msg::PoseStamped();
        geometry_msgs::msg::TransformStamped t;
        rclcpp::Time prev_time;
        bool first_cb = true;
        geometry_msgs::msg::Twist commandedTwist;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}