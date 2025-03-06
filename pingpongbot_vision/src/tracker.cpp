#include <mutex>
#include <memory>
#include <onnxruntime_cxx_api.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

class Tracker : public rclcpp::Node {
public:
    Tracker() : Node("tracker"), env(ORT_LOGGING_LEVEL_WARNING, "tracker"), session(nullptr) {
        // Declare and retrieve ONNX model path parameter
        this->declare_parameter<std::string>("onnx_model_path", "ppball_detection.onnx");
        std::string model_path = this->get_parameter("onnx_model_path").as_string();

        // Set up ONNX Runtime session
        Ort::SessionOptions session_options;
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        /// Define input and output node names
        const char* input_name = session->GetInputName(0, allocator);  // First input
        const char* output_name = session->GetOutputName(0, allocator);  // First output

        std::vector<const char*> input_node_names = {input_name};
        std::vector<const char*> output_node_names = {output_name};

        // Run inference
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);

        // Get and log model input shape
        auto input_dims = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        input_width = input_dims[2];  // Assuming [batch, channels, height, width]
        input_height = input_dims[3];

        // Initialize the publisher
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ball_pos", 10);

        // Subscribe to raw image and depth streams from the camera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", 10, 
            std::bind(&Tracker::imageCallback, this, std::placeholders::_1));

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
        // Resize the image to 640x640 (model input size)
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(640, 640));
    
        // Debug: Check the dimensions of the resized frame
        RCLCPP_INFO(this->get_logger(), "Resized frame size: %d x %d", resized_frame.cols, resized_frame.rows);
    
        // Convert to float and normalize (optional step depending on your model preprocessing)
        resized_frame.convertTo(resized_frame, CV_32F, 1.0 / 255.0);  // Normalize pixel values to [0, 1]
        
        // Flatten the image into a 1D vector of floats
        std::vector<float> input_tensor_values;
        for (int row = 0; row < resized_frame.rows; ++row) {
            for (int col = 0; col < resized_frame.cols; ++col) {
                cv::Vec3f pixel = resized_frame.at<cv::Vec3f>(row, col);
                input_tensor_values.push_back(pixel[0]);  // Blue channel
                input_tensor_values.push_back(pixel[1]);  // Green channel
                input_tensor_values.push_back(pixel[2]);  // Red channel
            }
        }
    
        // Debug: Check the size of the tensor values
        RCLCPP_INFO(this->get_logger(), "Input tensor size: %zu", input_tensor_values.size());
    
        // Verify that the tensor size matches the expected size
        if (input_tensor_values.size() != 640 * 640 * 3) {
            RCLCPP_ERROR(this->get_logger(), "Input tensor size mismatch: expected %d, got %zu", 640 * 640 * 3, input_tensor_values.size());
            return;
        }
    
        // Create the tensor
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::array<int64_t, 4> input_shape = {1, 3, 640, 640};  // [batch_size, channels, height, width]
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_values.size(),
            input_shape.data(), input_shape.size());
        
        // Define input and output node names
        std::vector<const char*> input_node_names = {"input"};  // Replace with the actual input name from your model
        std::vector<const char*> output_node_names = {"output"}; // Replace with the actual output name from your model
        
        // Run inference
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);
        
        // Get output data
        float* output_data = output_tensors.front().GetTensorMutableData<float>();
        
        // Extract (x, y) coordinates
        float x = output_data[0] * frame.cols;
        float y = output_data[1] * frame.rows;
        
        // Publish the position of the detected ball
        publishPosition(x, y);
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

        // Calculate 3D position
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

    // ONNX Runtime members
    Ort::Env env;
    std::unique_ptr<Ort::Session> session;
    int input_width, input_height;
    Ort::AllocatorWithDefaultOptions allocator; // Added for input/output name retrieval
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>());
    rclcpp::shutdown();
    return 0;
}
