#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class Commander : public rclcpp::Node {

    public:
        Commander() : Node("commander") {

            sub_ball_pos_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "ball_pos", 10, std::bind(&Commander::ballPosCallback, this, std::placeholders::_1));
        }

    private:
        void ballPosCallback(const geometry_msgs::msg::PointStamped & msg) {
            if (!first_bp_cb) {
                current_ball_pos = msg;
                prev_ball_pos = current_ball_pos;
            } else {
                prev_ball_pos = msg;
            }
        }

        geometry_msgs::msg::PointStamped current_ball_pos;
        geometry_msgs::msg::PointStamped prev_ball_pos;
        bool first_bp_cb = true;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_ball_pos_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Commander>());
    rclcpp::shutdown();
    return 0;
}