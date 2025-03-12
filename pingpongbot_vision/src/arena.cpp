#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"

class Arena : public rclcpp::Node {

    public:
        Arena() : Node("arena") {

            rclcpp::QoS qos(10);
            qos.reliable();
            qos.transient_local();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000), std::bind(&Arena::timerCallback, this));

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this, qos);
            tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this, qos);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void timerCallback() {
            try {
                t_robot = tf_buffer_->lookupTransform("camera_link", "robot", tf2::TimePointZero);
                t_table = tf_buffer_->lookupTransform("camera_link", "table", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(  // Use WARN instead of INFO for errors
                    this->get_logger(), 
                    "Could not transform %s to %s: %s",
                    odom_id.c_str(), base_id.c_str(), ex.what());
                return;
            }

            if (!table_set) {
                tf2::Quaternion q_table_tf2;
                tf2::fromMsg(t_table.transform.rotation, q_table_tf2);
                q_table_tf2 = q_table_tf2.normalized();

                double table_yaw, table_pitch, table_roll;
                tf2::getEulerYPR(q_table_tf2, table_yaw, table_pitch, table_roll);

                tf2::Quaternion q_table_tf2_adjusted;
                q_table_tf2_adjusted.setRPY(0.0, table_pitch, 0.0);

                t_table.transform.rotation = tf2::toMsg(q_table_tf2_adjusted);

                tf_static_broadcaster_->sendTransform(t_table);
                table_set = true;
            }

            

        }

        geometry_msgs::msg::TransformStamped t_robot;
        geometry_msgs::msg::TransformStamped t_table;
        bool table_set = false;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arena>());
    rclcpp::shutdown();
    return 0;
}
