#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "pingpongbot_driver/interface.hpp"
#include "pingpongbot_msgs/msg/wheel_speeds.hpp"
#include "pingpongbot_msgs/msg/wheel_angles.hpp"

class Driver : public rclcpp::Node {
    
    public:
        Driver() : Node("driver") {

            interface = std::make_shared<pingpongbot_driver::Interface>();

            // Check if driver_ is correctly initialized
            if (!interface) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize the Interface object.");
            }

            this->interface->resetEncoderPulses();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&Driver::timerCallback, this));

            wheel_angles_pub_ = this->create_publisher<pingpongbot_msgs::msg::WheelAngles>("wheel_angles", 10);

            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

            wheel_speeds_sub_ = this->create_subscription<pingpongbot_msgs::msg::WheelSpeeds>(
                "wheel_speeds", 10, std::bind(&Driver::wheelSpeedsCallback, this, std::placeholders::_1));

            heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
                "heartbeat", 10, std::bind(&Driver::heartbeatCallback, this, std::placeholders::_1));

            reset_encoder_srv_ = this->create_service<std_srvs::srv::Empty>(
                "reset_encoder", std::bind(&Driver::resetEncoder, this, std::placeholders::_1, std::placeholders::_2));
        }

        ~Driver() {
            RCLCPP_INFO(this->get_logger(), "Shutting down PingPongBotDriver...");
            this->interface.reset(); // Explicitly reset the driver before node shutdown
        }
    

    private:
        void timerCallback() {
            auto msg = this->interface->getWheelAngles();
            wheel_angles_pub_->publish(msg);
            // pingpongbot_msgs::msg::WheelSpeeds speeds;
            // speeds.u1 = 60;
            // speeds.u2 = 60;
            // speeds.u3 = 60;
            this->interface->setSpeeds(speeds);
            auto current_time = this->get_clock()->now();
            if ((current_time - last_heartbeat_time).seconds() > 0.175) {
                // RCLCPP_WARN(this->get_logger(), "Lost connection! Stopping wheels.");
                this->interface->zeroSpeeds();
                // driver->resetEncoderPulses();
            }

            pingpongbot_msgs::msg::IMU data = this->interface->getIMUData();

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = "base_footprint";
            imu_msg.angular_velocity.x = data.xg;
            imu_msg.angular_velocity.y = data.yg;
            imu_msg.angular_velocity.z = data.zg;
            imu_msg.linear_acceleration_covariance = {6.854e-8, 0, 0, 0, 6.854e-8, 0, 0, 0, 6.854e-8};
            imu_msg.linear_acceleration.x = data.xa;
            imu_msg.linear_acceleration.y = data.ya;
            imu_msg.linear_acceleration.z = data.za;
            imu_msg.angular_velocity_covariance = {1.500625e-6, 0, 0, 0, 1.500625e-6, 0, 0, 0, 1.500625e-6};

            yaw += data.zg*((current_time - prev_time).seconds());
            yaw_variance = gyro_noise_density * gyro_noise_density * ((current_time - prev_time).seconds()) * ((current_time - prev_time).seconds());

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);

            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            imu_msg.orientation_covariance = {yaw_variance, 0, 0, 0, 0, 0, 0, 0, 0};

            imu_pub_->publish(imu_msg);

            // static pingpongbot_msgs::msg::WheelAngles prev_angles;  // Store previous wheel angles

            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "Wheel Angles: θ1: %.6f, θ2: %.6f, θ3: %.6f", 
            //             msg.theta1, msg.theta2, msg.theta3);

            // // Compute and log delta values (change in wheel angles)
            // double delta_theta1 = msg.theta1 - prev_angles.theta1;
            // double delta_theta2 = msg.theta2 - prev_angles.theta2;
            // double delta_theta3 = msg.theta3 - prev_angles.theta3;

            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "ΔTheta: dθ1: %.6f, dθ2: %.6f, dθ3: %.6f",
            //             delta_theta1, delta_theta2, delta_theta3);

            // std::array<int8_t, 3> speedInts = driver->getSpeeds();

            // Update previous angles
            // RCLCPP_INFO(rclcpp::get_logger("wheel_logger"),
            //             "Motor Speeds: 1: %.6f, 2: %.6f, 3: %.6f", 
            //             speedInts[0], speedInts[1], speedInts[2]);
            // prev_angles = msg;
            prev_time = current_time;
        }

        void wheelSpeedsCallback(const pingpongbot_msgs::msg::WheelSpeeds & msg) {
            speeds = msg;
        }

        void heartbeatCallback(const std_msgs::msg::Empty & msg) {
            (void) msg;
            last_heartbeat_time = this->clock_.now();
        }

        void resetEncoder(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                this->interface->resetEncoderPulses();
        }

        std::shared_ptr<pingpongbot_driver::Interface> interface;
        pingpongbot_msgs::msg::WheelSpeeds speeds;
        rclcpp::Time last_heartbeat_time;
        double yaw = 0;
        double yaw_variance = 0;
        double gyro_noise_density = .0002618;
        rclcpp::Time prev_time = this->get_clock()->now();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock clock_;
        rclcpp::Publisher<pingpongbot_msgs::msg::WheelAngles>::SharedPtr wheel_angles_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Subscription<pingpongbot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_encoder_srv_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Driver>());
    rclcpp::shutdown();
    return 0;
}