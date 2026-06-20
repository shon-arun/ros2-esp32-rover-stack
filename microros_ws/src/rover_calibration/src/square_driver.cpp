#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class SquareDriver : public rclcpp::Node {
public:
    SquareDriver() : Node("square_driver"), state_(FORWARD), side_count_(0), pose_initialized_(false) {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&SquareDriver::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SquareDriver::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Square Driver Initialized. Waiting for EKF Odometry...");
    }

private:
    enum State { FORWARD, TURN, DONE };
    State state_;
    int side_count_;
    bool pose_initialized_;

    double current_x_, current_y_, current_yaw_;
    double start_x_, start_y_, start_yaw_;

    const double target_distance_ = 1.0;       // 1.0 meter square
    const double target_yaw_ = M_PI_2;         // 90 degrees in radians
    const double linear_speed_ = 0.1;          // m/s
    const double angular_speed_ = 3.0;         // rad/s

    double euler_from_quaternion(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        current_yaw_ = euler_from_quaternion(q.x, q.y, q.z, q.w);

        if (!pose_initialized_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_yaw_ = current_yaw_;
            pose_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial Pose acquired. Starting square...");
        }
    }

    void control_loop() {
        if (!pose_initialized_ || state_ == DONE) return;

        geometry_msgs::msg::Twist msg;

        if (state_ == FORWARD) {
            double distance = std::hypot(current_x_ - start_x_, current_y_ - start_y_);

            if (distance < target_distance_) {
                msg.linear.x = linear_speed_;
            } else {
                RCLCPP_INFO(this->get_logger(), "Finished Side %d. Turning...", side_count_ + 1);
                state_ = TURN;
                start_yaw_ = current_yaw_;
            }
        }
        else if (state_ == TURN) {
            double yaw_diff = current_yaw_ - start_yaw_;
            // Normalize angle to handle pi to -pi wraparound
            yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));

            if (std::abs(yaw_diff) < target_yaw_) {
                msg.angular.z = angular_speed_;
            } else {
                state_ = FORWARD;
                start_x_ = current_x_;
                start_y_ = current_y_;
                side_count_++;
                RCLCPP_INFO(this->get_logger(), "Turn complete. Driving straight...");

                if (side_count_ >= 4) {
                    RCLCPP_INFO(this->get_logger(), "Square complete. Shutting down motors.");
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    cmd_pub_->publish(msg);
                    state_ = DONE;
                    rclcpp::shutdown();
                    return;
                }
            }
        }
        cmd_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareDriver>());
    rclcpp::shutdown();
    return 0;
}
