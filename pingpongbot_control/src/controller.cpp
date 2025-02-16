#include <chrono>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "pingpongbot_msgs/msg/error.hpp"

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
            this->declare_parameter("min_speed", .75);

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
            min_speed = this->get_parameter("min_speed").as_double();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Controller::timerCallback, this));

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);

            error_pub_ = this->create_publisher<pingpongbot_msgs::msg::Error>("error", 10);

            reached_goal_pub_ = this->create_publisher<std_msgs::msg::Bool>("reached_goal", 10);

            goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "goal_pose", 10, std::bind(&Controller::goalPoseCallback, this, std::placeholders::_1));
            
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&Controller::onParamChange, this, std::placeholders::_1));
            

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

                if (!reached_goal && (distance_error < thresh_lin) && (std::abs(error_ang) < thresh_ang)) {

                    commandedTwist.linear.x = 0;
                    commandedTwist.linear.y = 0;
                    commandedTwist.angular.z = 0;
                    reached_goal = true;

                } else if (!reached_goal) {

                    accum_error_linx += error_linx * dt;
                    accum_error_liny += error_liny * dt;
                    accum_error_ang += error_ang * dt;

                    auto vx = (Kp_x * error_linx) + (Ki_x * accum_error_linx) + (Kd_x * ((error_linx - prev_error_linx)/dt));
                    auto vy = (Kp_y * error_liny) + (Ki_y * accum_error_liny) + (Kd_y * ((error_liny - prev_error_liny)/dt));
                    auto wz = (Kp_ang * error_ang) + (Ki_ang * accum_error_ang) + (Kd_ang * ((error_ang - prev_error_ang)/dt));

                    // if (vx < 0) {
                    //     vx = std::min(vx, -1 * min_speed);
                    // } else {
                    //     vx = std::max(vx, min_speed);
                    // }

                    // if (vy < 0) {
                    //     vy = std::min(vx, -1 * min_speed);
                    // } else {
                    //     vy = std::max(vx, min_speed);
                    // }

                    commandedTwist.linear.x = vx;
                    commandedTwist.linear.y = vy;
                    commandedTwist.angular.z = wz;
                    
                    if (count >= 100) {
                        error_msg.x_error = error_linx;
                        error_msg.y_error = error_liny;
                        error_msg.ang_error = error_ang;
                        error_pub_->publish(error_msg);
                        count = 0;
                    }
                    
                    prev_error_linx = error_linx;
                    prev_error_liny = error_liny;
                    prev_error_ang = error_ang;

                } else if (reached_goal && (distance_error > thresh_lin) && (std::abs(error_ang) > thresh_ang)){
                    reached_goal = false;
                }

            } else {
                first_cb = false;
            }
            prev_time = current_time;
            cmd_vel_pub_->publish(commandedTwist);
            heartbeat_pub_->publish(empty);
            reached_goal_msg.data = reached_goal;
            reached_goal_pub_->publish(reached_goal_msg);
            count++;
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

        rcl_interfaces::msg::SetParametersResult onParamChange(
            const std::vector<rclcpp::Parameter> & params) {
            for (const auto &param : params) {
                if (param.get_name() == "Kp_x") {
                    Kp_x = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kp_x: %.3f", Kp_x);
                } else if (param.get_name() == "Ki_x") {
                    Ki_x = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Ki_x: %.3f", Ki_x);
                } else if (param.get_name() == "Kd_x") {
                    Kd_x = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kd_x: %.3f", Kd_x);
                } else if (param.get_name() == "Kp_y") {
                    Kp_y = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kp_y: %.3f", Kp_y);
                } else if (param.get_name() == "Ki_y") {
                    Ki_y = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Ki_y: %.3f", Ki_y);
                } else if (param.get_name() == "Ki_x") {
                    Kd_y = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kd_y: %.3f", Kd_y);
                } else if (param.get_name() == "Kp_ang") {
                    Kp_ang = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kp_ang: %.3f", Kp_ang);
                } else if (param.get_name() == "Ki_ang") {
                    Ki_ang = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Ki_ang: %.3f", Kd_ang);
                } else if (param.get_name() == "Ki_ang") {
                    Kd_ang = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated Kd_ang: %.3f", Kd_ang);
                } else if (param.get_name() == "thresh_lin") {
                    thresh_lin = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated thresh_lin: %.3f", thresh_lin);
                } else if (param.get_name() == "thresh_ang") {
                    thresh_ang = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated thresh_ang: %.3f", thresh_ang);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

        double Kp_x, Ki_x, Kd_x;
        double Kp_y, Ki_y, Kd_y;
        double Kp_ang, Ki_ang, Kd_ang;
        double error_linx, prev_error_linx = 0, accum_error_linx = 0;
        double error_liny, prev_error_liny = 0, accum_error_liny = 0;
        double distance_error;
        double error_ang, prev_error_ang = 0, accum_error_ang = 0;
        double min_speed;
        pingpongbot_msgs::msg::Error error_msg;
        std::string odom_id;
        std::string base_id;
        double thresh_lin, thresh_ang;
        geometry_msgs::msg::PoseStamped currentGoal;
        geometry_msgs::msg::TransformStamped t;
        rclcpp::Time prev_time;
        bool first_cb = true;
        geometry_msgs::msg::Twist commandedTwist;
        std_msgs::msg::Empty empty;
        int count = 0;
        bool reached_goal = false;
        std_msgs::msg::Bool reached_goal_msg;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
        rclcpp::Publisher<pingpongbot_msgs::msg::Error>::SharedPtr error_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_goal_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}