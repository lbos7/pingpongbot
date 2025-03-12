#include <mutex>
#include <memory>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

class Tracker : public rclcpp::Node {
    
public:
    Tracker() : Node("tracker") {
        // Declare and retrieve the camera parameters
        this->declare_parameter<std::string>("camera_topic", "camera/camera/color/image_raw");
        std::string camera_topic = this->get_parameter("camera_topic").as_string();

        // Initialize the publisher for ball position
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ball_pos", 10);

        // Subscribe to raw image and depth streams from the camera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&Tracker::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/aligned_depth_to_color/image_raw", 10, 
            std::bind(&Tracker::depthCallback, this, std::placeholders::_1));

        // Set up RealSense camera
        cfg.enable_stream(RS2_STREAM_COLOR);
        cfg.enable_stream(RS2_STREAM_DEPTH);
        auto profile = pipe.start(cfg);
        auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        intrinsics = stream.get_intrinsics();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        detectBall(frame);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        depth_frame = cv_bridge::toCvCopy(msg, "16UC1")->image;
    }

    void detectBall(cv::Mat &frame) {
        // Convert the image to HSV
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        // Define the HSV range for the orange ping pong ball (as given)
        cv::Scalar lower_orange(8, 164, 197);  // Lower bound of the HSV range
        cv::Scalar upper_orange(25, 255, 255); // Upper bound of the HSV range

        // Apply the mask to extract the ball based on the HSV thresholding
        cv::Mat mask;
        cv::inRange(hsv_frame, lower_orange, upper_orange, mask);

        // Find contours of the masked image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // If contours are found, calculate the ball's position
        if (!contours.empty()) {
            // Get the largest contour
            size_t largest_contour_index = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    largest_contour_index = i;
                }
            }

            // Calculate the centroid of the largest contour
            cv::Moments moments = cv::moments(contours[largest_contour_index]);
            int cX = static_cast<int>(moments.m10 / moments.m00);
            int cY = static_cast<int>(moments.m01 / moments.m00);

            // Publish the ball position
            publishPosition(cX, cY);
            
            // Optionally: Draw the contours and centroid on the frame for debugging
            cv::drawContours(frame, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
        }

        // Display the result
        cv::imshow("Ball Tracker", frame);
        cv::waitKey(1);
    }

    void publishPosition(float x, float y) {
        std::lock_guard<std::mutex> lock(mutex);
        if (depth_frame.empty()) return;

        if (x < 0 || x >= depth_frame.cols || y < 0 || y >= depth_frame.rows) {
            RCLCPP_WARN(this->get_logger(), "Detected position out of bounds!");
            return;
        }

        // Get depth value
        float depth = depth_frame.at<uint16_t>(static_cast<int>(y), static_cast<int>(x)) * 0.001f;

        // Calculate 3D position using depth and camera intrinsics
        float point3D[3];
        float pixel[2] = {x, y};
        rs2_deproject_pixel_to_point(point3D, &intrinsics, pixel, depth);

        auto msg = geometry_msgs::msg::PointStamped();
        msg.header.stamp = this->now();
        msg.point.x = point3D[0];
        msg.point.y = point3D[1];
        msg.point.z = point3D[2];
        position_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    cv::Mat depth_frame;
    rs2_intrinsics intrinsics;
    std::mutex mutex;
    rs2::pipeline pipe;
    rs2::config cfg;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>());
    rclcpp::shutdown();
    return 0;
}
