#include <mutex>
#include <memory>
#include <string>

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

            this->declare_parameter("model_path", "");
            model_path = this->get_parameter("model_path").as_string();

            position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ball_pos", 10);

            // Subscribe to raw image and depth streams from the camera
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/camera/color/image_raw", 10, 
                std::bind(&Tracker::imageCallback, this, std::placeholders::_1));

            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/camera/aligned_depth_to_color/image_raw", 10, 
                std::bind(&Tracker::depthCallback, this, std::placeholders::_1));

            // Load the pre-trained ONNX model
            net = cv::dnn::readNetFromONNX(model_path);

            // Set up RealSense camera
            cfg.enable_stream(RS2_STREAM_COLOR);
            cfg.enable_stream(RS2_STREAM_DEPTH);
            auto profile = pipe.start(cfg);
            auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            intrinsics = stream.get_intrinsics();
        }
    
    private:
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex);
            if (depth_frame.empty()) return; // Skip if depth frame is not available
            
            // Convert the incoming image to OpenCV format
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;


            int target_width = 640;
            int target_height = 640;

            // Ensure the dimensions are divisible by 9
            target_width = (target_width / 9) * 9;
            target_height = (target_height / 9) * 9;
            
            // Resize the image to 640x640
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(target_width, target_height));  // Resize to match model input size
            
            // Convert the image to a blob for the model
            cv::Mat blob = cv::dnn::blobFromImage(resized_frame, 1.0 / 255.0, cv::Size(target_width, target_height), cv::Scalar(), true, false);
            net.setInput(blob);

            // Perform inference and get the output
            cv::Mat output = net.forward();
            // Process the output (e.g., detecting ball's position)
            float x = output.at<float>(0, 0) * frame.cols;
            float y = output.at<float>(0, 1) * frame.rows;
            
            publishPosition(x, y);
        }
    
        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex);
            depth_frame = cv_bridge::toCvCopy(msg, "16UC1")->image;
        }
    
        void publishPosition(float x, float y) {
            std::lock_guard<std::mutex> lock(mutex);
            if (depth_frame.empty()) return;
            
            // Retrieve depth value from the depth frame
            float depth = depth_frame.at<uint16_t>(static_cast<int>(y), static_cast<int>(x)) * 0.001f;
            
            // De-project the pixel coordinates to 3D space
            float point3D[3];
            float pixel[2] = {x, y};
            rs2_deproject_pixel_to_point(point3D, &intrinsics, pixel, depth);
            
            // Publish the 3D position
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
        std::string model_path;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    };
    
    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Tracker>());
        rclcpp::shutdown();
        return 0;
    }
