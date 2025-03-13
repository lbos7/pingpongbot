#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Commander::timerCallback, this));

            sub_ball_pos_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "ball_pos", 10, std::bind(&Commander::ballPosCallback, this, std::placeholders::_1));

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void timerCallback() {

        }

        void ballPosCallback(const geometry_msgs::msg::PointStamped & msg) {
            try {
                t_camera_center = tf_buffer_->lookupTransform("camera_link", "table_center", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(  // Use WARN instead of INFO for errors
                    this->get_logger(), 
                    "Could not transform: %s",
                    ex.what());
                return;
            }

            tf2::doTransform(msg, trans_ball_pos, t_camera_center);

            if (!first_bp_cb) {
                current_ball_pos = trans_ball_pos;
                prev_ball_pos = current_ball_pos;
            } else {
                prev_ball_pos = trans_ball_pos;
                first_bp_cb = false;
            }
        }

        geometry_msgs::msg::PointStamped current_ball_pos, prev_ball_pos, trans_ball_pos;
        geometry_msgs::msg::TransformStamped t_camera_center;
        bool first_bp_cb = true;
        rclcpp::TimerBase::SharedPtr timer_;
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