#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class ApriltagHandler : public rclcpp::Node {

    public:
        ApriltagHandler() : Node("apriltag_handler") {

            detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "detections", 10, std::bind(&ApriltagHandler::detectionCallback, this, std::placeholders::_1));
        }

    private:
        void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray & msg) {
            currentDetection = msg;
        }

        apriltag_msgs::msg::AprilTagDetectionArray currentDetection;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
        
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagHandler>());
    rclcpp::shutdown();
    return 0;
}
