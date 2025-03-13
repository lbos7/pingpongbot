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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Arena : public rclcpp::Node {

    public:
        Arena() : Node("arena") {

            this->declare_parameter("table_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("table_length", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("marker_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("marker_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
            this->declare_parameter("table_offset", rclcpp::ParameterType::PARAMETER_DOUBLE);

            table_width = this->get_parameter("table_width").as_double();
            table_length = this->get_parameter("table_length").as_double();
            marker_width = this->get_parameter("marker_width").as_double();
            marker_height = this->get_parameter("marker_height").as_double();
            table_offset = this->get_parameter("table_offset").as_double();

            rclcpp::QoS qos(10);
            qos.reliable();
            qos.transient_local();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000), std::bind(&Arena::timerCallback, this));

            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", qos);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this, qos);

            tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this, qos);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void timerCallback() {
            try {
                t_robot = tf_buffer_->lookupTransform("camera_link", "robot_tag", tf2::TimePointZero);
                t_table = tf_buffer_->lookupTransform("camera_link", "table_tag", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(  // Use WARN instead of INFO for errors
                    this->get_logger(), 
                    "Could not transform: %s",
                    ex.what());
                return;
            }

            if (!table_set) {
                tf2::Quaternion q_table_tf2;
                tf2::fromMsg(t_table.transform.rotation, q_table_tf2);
                q_table_tf2 = q_table_tf2.normalized();

                double table_yaw, table_pitch, table_roll;
                tf2::getEulerYPR(q_table_tf2, table_yaw, table_pitch, table_roll);

                tf2::Quaternion q_table_tf2_adjusted;
                q_table_tf2_adjusted.setRPY(0.0, 0.0, table_yaw);

                t_table.transform.rotation = tf2::toMsg(q_table_tf2_adjusted);

                t_table.child_frame_id = "table";
                tf_static_broadcaster_->sendTransform(t_table);

                geometry_msgs::msg::TransformStamped corner_to_center;
                corner_to_center.header.stamp = this->get_clock()->now();
                corner_to_center.header.frame_id = "table";
                corner_to_center.child_frame_id = "table_center";
                corner_to_center.transform.translation.x = table_width/2;
                corner_to_center.transform.translation.y = table_length/4;
                corner_to_center.transform.translation.z = -table_offset;

                tf2::Quaternion corner_to_center_q;
                corner_to_center_q.setRPY(0.0, 0.0, M_PI);
                corner_to_center.transform.rotation = tf2::toMsg(corner_to_center_q);

                tf_static_broadcaster_->sendTransform(corner_to_center);

                visualization_msgs::msg::Marker m_back;
                visualization_msgs::msg::Marker m_left;
                visualization_msgs::msg::Marker m_right;
                visualization_msgs::msg::Marker m_front;

                m_back.header.stamp = this->get_clock()->now();
                m_back.header.frame_id = "table_center";
                m_back.id = 0;
                m_back.type = visualization_msgs::msg::Marker::CUBE;
                m_back.action = visualization_msgs::msg::Marker::ADD;
                m_back.scale.x = table_width + 2*marker_width;
                m_back.scale.y = marker_width;
                m_back.scale.z = marker_height;
                m_back.pose.position.y = table_length/4 + marker_width/2; 
                m_back.color.r = 1.0;
                m_back.color.g = 0.0;
                m_back.color.b = 0.0;
                m_back.color.a = 1.0;

                m_left.header.stamp = this->get_clock()->now();
                m_left.header.frame_id = "table_center";
                m_left.id = 1;
                m_left.type = visualization_msgs::msg::Marker::CUBE;
                m_left.action = visualization_msgs::msg::Marker::ADD;
                m_left.scale.x = marker_width;
                m_left.scale.y = table_length/2 + 2*marker_width;
                m_left.scale.z = marker_height;
                m_left.pose.position.x = table_width/2 + marker_width/2; 
                m_left.color.r = 1.0;
                m_left.color.g = 0.0;
                m_left.color.b = 0.0;
                m_left.color.a = 1.0;

                m_right.header.stamp = this->get_clock()->now();
                m_right.header.frame_id = "table_center";
                m_right.id = 3;
                m_right.type = visualization_msgs::msg::Marker::CUBE;
                m_right.action = visualization_msgs::msg::Marker::ADD;
                m_right.scale.x = marker_width;
                m_right.scale.y = table_length/2 + 2*marker_width;
                m_right.scale.z = marker_height;
                m_right.pose.position.x = -table_width/2 - marker_width/2; 
                m_right.color.r = 1.0;
                m_right.color.g = 0.0;
                m_right.color.b = 0.0;
                m_right.color.a = 1.0;

                m_front.header.stamp = this->get_clock()->now();
                m_front.header.frame_id = "table_center";
                m_front.id = 2;
                m_front.type = visualization_msgs::msg::Marker::CUBE;
                m_front.action = visualization_msgs::msg::Marker::ADD;
                m_front.scale.x = table_width + 2*marker_width;
                m_front.scale.y = marker_width;
                m_front.scale.z = marker_height;
                m_front.pose.position.y = -table_length/4 - marker_width/2; 
                m_front.color.r = 1.0;
                m_front.color.g = 0.0;
                m_front.color.b = 0.0;
                m_front.color.a = 1.0;

                visualization_msgs::msg::MarkerArray m_array;
                m_array.markers = {m_back, m_left, m_front, m_right};

                marker_pub_->publish(m_array);

                tf2::Quaternion q_robot_tf2;
                tf2::fromMsg(t_robot.transform.rotation, q_robot_tf2);
                q_robot_tf2 = q_robot_tf2.normalized();

                double robot_yaw, robot_pitch, robot_roll;
                tf2::getEulerYPR(q_robot_tf2, robot_yaw, robot_pitch, robot_roll);

                tf2::Quaternion q_robot_tf2_adjusted;
                q_robot_tf2_adjusted.setRPY(0.0, robot_pitch, robot_yaw);

                t_robot.transform.rotation = tf2::toMsg(q_robot_tf2_adjusted);

                t_robot.child_frame_id = "robot";
                tf_broadcaster_->sendTransform(t_robot);

                geometry_msgs::msg::TransformStamped robot_to_odom;
                robot_to_odom.header.stamp = this->get_clock()->now();
                robot_to_odom.header.frame_id = "robot";
                robot_to_odom.child_frame_id = "odom";
                robot_to_odom.transform.translation.y = -.0165;
                robot_to_odom.transform.translation.z = -.237;

                tf_static_broadcaster_->sendTransform(robot_to_odom);

                table_set = true;
            }

            tf2::Quaternion q_robot_tf2;
            tf2::fromMsg(t_robot.transform.rotation, q_robot_tf2);
            q_robot_tf2 = q_robot_tf2.normalized();

            double robot_yaw, robot_pitch, robot_roll;
            tf2::getEulerYPR(q_robot_tf2, robot_yaw, robot_pitch, robot_roll);

            tf2::Quaternion q_robot_tf2_adjusted;
            q_robot_tf2_adjusted.setRPY(0.0, robot_pitch, robot_yaw);

            t_robot.transform.rotation = tf2::toMsg(q_robot_tf2_adjusted);

            t_robot.child_frame_id = "robot";
            tf_broadcaster_->sendTransform(t_robot);

        }

        double table_width, table_length, marker_width, marker_height, table_offset;
        geometry_msgs::msg::TransformStamped t_robot;
        geometry_msgs::msg::TransformStamped t_table;
        bool table_set = false;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
