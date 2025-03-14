#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

class Commander : public rclcpp::Node {

    public:
        Commander() : Node("commander") {
            this->declare_parameter("paddle_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("d", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("table_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("table_length", rclcpp::ParameterType::PARAMETER_DOUBLE);

            paddle_height = this->get_parameter("paddle_height").as_double();
            d = this->get_parameter("d").as_double();
            table_width = this->get_parameter("table_width").as_double();
            table_length = this->get_parameter("table_length").as_double();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Commander::timerCallback, this));

            pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

            sub_ball_pos_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "ball_pos", 10, std::bind(&Commander::ballPosCallback, this, std::placeholders::_1));

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void timerCallback() {
        }

        void ballPosCallback(const geometry_msgs::msg::PointStamped & msg) {
            if (!looked_up) {
                try {
                    t_camera_center = tf_buffer_->lookupTransform("camera_color_optical_frame", "table_center", tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    // RCLCPP_WARN(  // Use WARN instead of INFO for errors
                    //     this->get_logger(), 
                    //     "Could not transform: %s",
                    //     ex.what());
                    return;
                }
                looked_up = true;
            }


            tf2::doTransform(msg, trans_ball_pos, t_camera_center);

            if (!first_bp_cb) {
                current_time = this->get_clock()->now();
                auto dt = (current_time - prev_time).seconds();
                current_ball_pos.header.stamp = current_time;
                current_ball_pos.header.frame_id = "table_center";
                current_ball_pos.point.x = trans_ball_pos.point.x;
                current_ball_pos.point.y = trans_ball_pos.point.z;
                current_ball_pos.point.z = trans_ball_pos.point.y;
                // RCLCPP_INFO(this->get_logger(),
                // "Ball position: x = %f, y = %f, z = %f, frame_id = %s",
                // msg.point.x, msg.point.y, msg.point.z,
                // msg.header.frame_id.c_str());
                // RCLCPP_INFO(this->get_logger(),
                // "Transformed ball position: x = %f, y = %f, z = %f, frame_id = %s",
                // trans_ball_pos.point.x, trans_ball_pos.point.y, trans_ball_pos.point.z,
                // trans_ball_pos.header.frame_id.c_str());

                vx = (current_ball_pos.point.x - prev_ball_pos.point.x)/dt;
                vy = (current_ball_pos.point.y - prev_ball_pos.point.y)/dt;
                vz = (current_ball_pos.point.z - prev_ball_pos.point.z)/dt;
                double discriminant = vz*vz + 2*g*(paddle_height - current_ball_pos.point.z);

                if (discriminant < 0) {
                    goal_pose.header.stamp = current_time;
                    goal_pose.header.frame_id = "table_center";
                    goal_pose.pose.position.x = current_ball_pos.point.x;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 0;

                } else if (discriminant > 0) {
                    double t_hit = (vz + std::sqrt(discriminant))/g;
                    contact_guess.header.stamp = current_time;
                    contact_guess.header.frame_id = "table_center";
                    contact_guess.point.x = current_ball_pos.point.x + vx*t_hit;
                    contact_guess.point.y = current_ball_pos.point.y + vy*t_hit;

                    goal_pose.header.stamp = current_time;
                    goal_pose.header.frame_id = "table_center";
                    // goal_pose.pose.position = contact_guess.point;
                    goal_pose.pose.position.x = current_ball_pos.point.x;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 0;
                }

                if (goal_pose.pose.position.x > (table_width/2 - d)) {
                    goal_pose.pose.position.x = (table_width/2 - d);
                } else if (goal_pose.pose.position.x < (table_width/2 - d)*-1) {
                    goal_pose.pose.position.x = (table_width/2 - d)*-1;
                }

                if (goal_pose.pose.position.y > (table_length/4 - d)) {
                    goal_pose.pose.position.y = (table_length/4 - d);
                } else if (goal_pose.pose.position.y < (table_length/4 - d)*-1) {
                    goal_pose.pose.position.y = (table_length/4 - d)*-1;
                }

                pub_goal_pose_->publish(goal_pose);

                prev_ball_pos = current_ball_pos;
                prev_time = current_time;
            } else {
                prev_time = this->get_clock()->now();
                prev_ball_pos = trans_ball_pos;
                first_bp_cb = false;
            }
        }

        geometry_msgs::msg::PointStamped current_ball_pos, prev_ball_pos, trans_ball_pos, contact_guess;
        geometry_msgs::msg::TransformStamped t_camera_center;
        geometry_msgs::msg::PoseStamped goal_pose;
        bool first_bp_cb = true, looked_up = false;
        double vx = 0, vy = 0, vz = 0, g = 9.81, paddle_height, d, table_width, table_length;
        rclcpp::Time current_time, prev_time;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_ball_pos_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Commander>());
    rclcpp::shutdown();
    return 0;
}