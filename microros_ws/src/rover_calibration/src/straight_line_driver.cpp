#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

class StraightLineDriver : public rclcpp::Node {
public:
    StraightLineDriver() : Node("straight_line_driver"), state_(WAITING_FOR_ODOM), pose_initialized_(false), v_cmd_(0.0) {
        // --- Parameters ---
        this->declare_parameter("target_distance", 1.0);       
        this->declare_parameter("max_velocity", 0.2);          
        this->declare_parameter("max_acceleration", 0.2);     
        this->declare_parameter("kp_yaw", 1.5);                
        this->declare_parameter("max_angular_velocity", 1.0); 
        this->declare_parameter("tolerance", 0.005);           
        this->declare_parameter("min_sustainable_vel", 0.08);  

        target_distance_ = this->get_parameter("target_distance").as_double();
        max_vel_ = this->get_parameter("max_velocity").as_double();
        max_accel_ = this->get_parameter("max_acceleration").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        min_sustainable_vel_ = this->get_parameter("min_sustainable_vel").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&StraightLineDriver::odom_callback, this, std::placeholders::_1));

        // Start clocks
        last_time_ = this->get_clock()->now();
        last_odom_time_ = last_time_;

        // 50 Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&StraightLineDriver::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Professional Straight Line Planner Initialized. Waiting for EKF Odometry...");
    }

private:
    enum State { WAITING_FOR_ODOM, INIT, DRIVING, DONE };
    State state_;
    bool pose_initialized_;

    double current_x_, current_y_, current_yaw_;
    double start_x_, start_y_, start_yaw_;
    double v_cmd_;

    double target_distance_, max_vel_, max_accel_, kp_yaw_, max_angular_velocity_, tolerance_, min_sustainable_vel_;
    
    rclcpp::Time last_time_;
    rclcpp::Time last_odom_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Fast quaternion to yaw calculation
    inline double euler_from_quaternion(double x, double y, double z, double w) const {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    // O(1) angle normalization using std::remainder
    inline double normalize_angle(double angle) const {
        return std::remainder(angle, 2.0 * M_PI);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = this->get_clock()->now();
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        current_yaw_ = euler_from_quaternion(q.x, q.y, q.z, q.w);

        if (!pose_initialized_) {
            pose_initialized_ = true;
            state_ = INIT;
        }
    }

    void publish_stop() {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_pub_->publish(stop_msg);
        v_cmd_ = 0.0;
    }

    void control_loop() {
        if (state_ == WAITING_FOR_ODOM) return;

        // Ensure we cleanly hold the 0 state and gracefully shut down the node
        if (state_ == DONE) {
            publish_stop();
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Ensure ROS transmits
            rclcpp::shutdown();
            return;
        }

        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Ignore massive leaps in time (e.g., waking from sleep/breakpoints) or division by zero
        if (dt <= 0.0 || dt > 1.0) return;

        // --- SAFETY WATCHDOG ---
        // If odometry hasn't been received in 0.5 seconds, halt the motors
        if ((now - last_odom_time_).seconds() > 0.5) {
            publish_stop();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Odometry timeout! EKF stopped publishing. Halting rover.");
            return;
        }

        if (state_ == INIT) {
            // Immediate exit if target is 0
            if (std::abs(target_distance_) < tolerance_) {
                state_ = DONE;
                return;
            }

            start_x_ = current_x_;
            start_y_ = current_y_;
            start_yaw_ = current_yaw_;
            state_ = DRIVING;
            RCLCPP_INFO(this->get_logger(), "Starting Trajectory: Distance = %.2fm", target_distance_);
        }

        if (state_ == DRIVING) {
            double dx = current_x_ - start_x_;
            double dy = current_y_ - start_y_;
            
            // Project current position onto the initial heading vector. 
            // Negative values naturally occur if moving backward.
            double distance_traveled = (dx * std::cos(start_yaw_)) + (dy * std::sin(start_yaw_));
            double direction = (target_distance_ >= 0.0) ? 1.0 : -1.0;

            // Robust Directional Line-Crossing Check
            bool reached_target = (direction > 0.0) 
                ? (distance_traveled >= target_distance_ - tolerance_)
                : (distance_traveled <= target_distance_ + tolerance_);

            if (reached_target) {
                state_ = DONE;
                RCLCPP_INFO(this->get_logger(), "Target perfectly reached. Traveled: %.4fm. Shutting down.", distance_traveled);
                return;
            }

            // True Distance Remaining calculation (avoids absolute value shrinkage errors)
            double distance_remaining = std::abs(target_distance_ - distance_traveled);

            // Kinematic Velocity Profiling 
            double v_max_kinematic = std::sqrt(2.0 * max_accel_ * distance_remaining);
            double v_target_accel = v_cmd_ + (max_accel_ * dt);
            
            double v_ideal = std::min({max_vel_, v_target_accel, v_max_kinematic});
            
            // Maintain minimum sustainable speed until the target line is physically crossed. 
            v_ideal = std::max(v_ideal, min_sustainable_vel_);
            
            // Final safety clamp using standard library
            v_cmd_ = std::clamp(v_ideal, 0.0, max_vel_);

            // Active Heading Correction
            double yaw_error = normalize_angle(start_yaw_ - current_yaw_);
            double angular_cmd = std::clamp(kp_yaw_ * yaw_error, -max_angular_velocity_, max_angular_velocity_); 

            // Publish Command
            geometry_msgs::msg::Twist msg;
            msg.linear.x = v_cmd_ * direction; 
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