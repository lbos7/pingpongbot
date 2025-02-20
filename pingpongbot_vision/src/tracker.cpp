#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

class Tracker : public rclcpp::Node {

    public:
    Tracker() : Node("tracker") {

            position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ball_pos", 10);

            // Subscribe to raw image and depth streams from the camera
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/camera/color/image_raw", 10, 
                std::bind(&Tracker::imageCallback, this, std::placeholders::_1));

            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/camera/aligned_depth_to_color/image_raw", 10, 
                std::bind(&Tracker::depthCallback, this, std::placeholders::_1));

            // Load the pre-trained ONNX model
            net = cv::dnn::readNetFromONNX("ppball_detection.onnx");

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
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(224, 224), cv::Scalar(), true, false);
            net.setInput(blob);
            cv::Mat output = net.forward();
            
            float x = output.at<float>(0, 0) * frame.cols;
            float y = output.at<float>(0, 1) * frame.rows;
            
            publishPosition(x, y);
        }
    
        void publishPosition(float x, float y) {
            std::lock_guard<std::mutex> lock(mutex);
            if (depth_frame.empty()) return;
            
            float depth = depth_frame.at<uint16_t>(static_cast<int>(y), static_cast<int>(x)) * 0.001f;
            
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
        cv::dnn::Net net;
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
