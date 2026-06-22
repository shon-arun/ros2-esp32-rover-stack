#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>

class StraightLineDriver : public rclcpp::Node {
public:
    StraightLineDriver() : Node("straight_line_driver"), state_(WAITING_FOR_ODOM), v_cmd_(0.0), pose_initialized_(false), stuck_cycles_(0) {
        // --- Parameters ---
        this->declare_parameter("target_distance", 1.0);       // Meters
        this->declare_parameter("max_velocity", 0.2);          // m/s
        this->declare_parameter("max_acceleration", 0.15);     // m/s^2
        this->declare_parameter("kp_yaw", 1.5);                // P-Gain for heading correction
        this->declare_parameter("tolerance", 0.005);           // 5mm stop tolerance
        this->declare_parameter("min_sustainable_vel", 0.08);  // Hardware stall floor (m/s)

        target_distance_ = this->get_parameter("target_distance").as_double();
        max_vel_ = this->get_parameter("max_velocity").as_double();
        max_accel_ = this->get_parameter("max_acceleration").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        min_sustainable_vel_ = this->get_parameter("min_sustainable_vel").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribe to the EKF filtered odometry for high precision
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&StraightLineDriver::odom_callback, this, std::placeholders::_1));

        // 50Hz Control Loop
        dt_ = 0.02; 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&StraightLineDriver::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Professional Straight Line Planner Initialized. Waiting for EKF Odometry...");
    }

private:
    enum State { WAITING_FOR_ODOM, INIT, DRIVING, DONE };
    State state_;
    bool pose_initialized_;
    int stuck_cycles_;

    double current_x_, current_y_, current_yaw_;
    double start_x_, start_y_, start_yaw_;
    double v_cmd_;

    double target_distance_, max_vel_, max_accel_, kp_yaw_, tolerance_, min_sustainable_vel_, dt_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Helper: Convert Quaternion to Euler Yaw
    double euler_from_quaternion(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Helper: Keep angles within -PI to PI
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        current_yaw_ = euler_from_quaternion(q.x, q.y, q.z, q.w);

        if (!pose_initialized_) {
            pose_initialized_ = true;
            state_ = INIT;
        }
    }

    void control_loop() {
        if (state_ == WAITING_FOR_ODOM || state_ == DONE) return;

        if (state_ == INIT) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_yaw_ = current_yaw_;
            state_ = DRIVING;
            RCLCPP_INFO(this->get_logger(), "Starting Trajectory: Distance = %.2fm", target_distance_);
        }

        if (state_ == DRIVING) {
            double distance_traveled = std::hypot(current_x_ - start_x_, current_y_ - start_y_);
            double distance_remaining = target_distance_ - distance_traveled;

            geometry_msgs::msg::Twist msg;

            // 1. Check for Perfect Stop Condition
            if (distance_remaining <= tolerance_) {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                cmd_pub_->publish(msg);
                state_ = DONE;
                RCLCPP_INFO(this->get_logger(), "Target perfectly reached. Traveled: %.4fm. Shutting down.", distance_traveled);
                rclcpp::shutdown();
                return;
            }

            // 2. Kinematic Velocity Profiling (Trapezoidal / S-Curve approximation)
            double v_max_kinematic = std::sqrt(2.0 * max_accel_ * std::max(0.0, distance_remaining));
            double v_target_accel = v_cmd_ + max_accel_ * dt_;
            
            // Calculate the ideal kinematic speed
            double v_ideal = std::min({max_vel_, v_target_accel, v_max_kinematic});
            v_ideal = std::max(v_ideal, 0.0); 

            // 3. Hardware Cutoff & Settling Watchdog
            if (v_ideal < min_sustainable_vel_) {
                v_cmd_ = 0.0; // Cut power, let momentum coast it in
                stuck_cycles_++;
                
                // If it has been coasting/stopped for 1 full second (50 cycles)
                if (stuck_cycles_ > 50) {
                    RCLCPP_WARN(this->get_logger(), "Settled short at %.3fm due to friction. Terminating cleanly.", distance_traveled);
                    state_ = DONE;
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    cmd_pub_->publish(msg);
                    rclcpp::shutdown();
                    return;
                }
            } else {
                v_cmd_ = v_ideal;
                stuck_cycles_ = 0; // Reset watchdog as long as we are driving normally
            }

            // 4. Active Heading Correction
            double yaw_error = normalize_angle(start_yaw_ - current_yaw_);
            double angular_cmd = kp_yaw_ * yaw_error;

            // 5. Publish Command
            msg.linear.x = v_cmd_;
            msg.angular.z = angular_cmd;
            cmd_pub_->publish(msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StraightLineDriver>());
    rclcpp::shutdown();
    return 0;
}