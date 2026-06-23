#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h> // [UPDATE] Added tf2 utilities for robust quaternion math
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // [FIX] Added bridge for geometry_msgs to tf2 conversion
#include <cmath>
#include <algorithm>
#include <chrono>

class RotationDriver : public rclcpp::Node {
public:
    RotationDriver() : Node("rotation_driver"), state_(WAITING_FOR_ODOM), pose_initialized_(false), yaw_traveled_(0.0), w_cmd_(0.0) {
        // --- Parameters ---
        this->declare_parameter("target_angle", 1.5708);       
        this->declare_parameter("max_angular_velocity", 1.0);          
        this->declare_parameter("max_angular_acceleration", 0.5);     
        this->declare_parameter("tolerance", 0.02);           
        this->declare_parameter("min_sustainable_angular_vel", 0.15);  

        target_angle_ = this->get_parameter("target_angle").as_double();
        
        // [UPDATE] Parameter Sanitization: Enforce absolute values for safety constraints
        max_vel_ = std::abs(this->get_parameter("max_angular_velocity").as_double());
        max_accel_ = std::abs(this->get_parameter("max_angular_acceleration").as_double());
        tolerance_ = std::abs(this->get_parameter("tolerance").as_double());
        min_sustainable_vel_ = std::abs(this->get_parameter("min_sustainable_angular_vel").as_double());

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&RotationDriver::odom_callback, this, std::placeholders::_1));

        // Start clocks
        last_time_ = this->get_clock()->now();
        last_odom_time_ = last_time_;

        // 50 Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RotationDriver::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Professional Rotation Planner Initialized. Waiting for EKF Odometry...");
    }

private:
    // [UPDATE] Added SETTLING state to counter inertia
    enum State { WAITING_FOR_ODOM, INIT, ROTATING, SETTLING, DONE };
    State state_;
    bool pose_initialized_;

    double current_yaw_;
    double yaw_traveled_; 
    double w_cmd_; 

    double target_angle_, max_vel_, max_accel_, tolerance_, min_sustainable_vel_;
    
    rclcpp::Time last_time_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time settling_start_time_; // Tracks how long we've been in the settling phase

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // [UPDATE] O(1) angle normalization using std::remainder, avoiding M_PI for portability
    inline double normalize_angle(double angle) const {
        static const double TWO_PI = 2.0 * std::acos(-1.0);
        return std::remainder(angle, TWO_PI);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = this->get_clock()->now();

        // [UPDATE] Replaced custom math with robust tf2 utility
        double new_yaw = tf2::getYaw(msg->pose.pose.orientation);

        if (!pose_initialized_) {
            current_yaw_ = new_yaw;
            pose_initialized_ = true;
            state_ = INIT;
        } else {
            // Calculate the shortest delta to handle pi/-pi wrap-around
            double delta_yaw = normalize_angle(new_yaw - current_yaw_);
            current_yaw_ = new_yaw;
            
            // Accumulate unwrapped travel distance during active phases
            if (state_ == ROTATING || state_ == SETTLING) {
                yaw_traveled_ += delta_yaw;
            }
        }
    }

    void publish_stop() {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_pub_->publish(stop_msg);
        w_cmd_ = 0.0;
    }

    void control_loop() {
        if (state_ == WAITING_FOR_ODOM) return;

        if (state_ == DONE) {
            publish_stop();
            // [FIX] Sleep briefly to ensure the stop message transmits, then cleanly shutdown the node.
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(this->get_logger(), "Rotation sequence complete. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 1.0) return;

        // --- SAFETY WATCHDOG ---
        if ((now - last_odom_time_).seconds() > 0.5) {
            publish_stop();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Odometry timeout! EKF stopped publishing. Halting rover.");
            return;
        }

        if (state_ == INIT) {
            if (std::abs(target_angle_) < tolerance_) {
                state_ = DONE;
                return;
            }

            yaw_traveled_ = 0.0;
            state_ = ROTATING;
            // Removed M_PI here to keep portability, calculated degree equivalent dynamically
            RCLCPP_INFO(this->get_logger(), "Starting Rotation: Target Angle = %.2f rad (%.1f deg)", 
                        target_angle_, target_angle_ * 180.0 / std::acos(-1.0));
        }

        if (state_ == ROTATING) {
            double direction = (target_angle_ >= 0.0) ? 1.0 : -1.0;

            bool reached_target = (direction > 0.0) 
                ? (yaw_traveled_ >= target_angle_ - tolerance_)
                : (yaw_traveled_ <= target_angle_ + tolerance_);

            if (reached_target) {
                // [UPDATE] Enter settling phase to damp momentum instead of immediately stopping
                state_ = SETTLING;
                settling_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Target boundary crossed. Entering 0.5s settling phase.");
                return;
            }

            double angle_remaining = std::abs(target_angle_ - yaw_traveled_);
            double w_max_kinematic = std::sqrt(2.0 * max_accel_ * angle_remaining);
            double w_target_accel = w_cmd_ + (max_accel_ * dt);
            
            double w_ideal = std::min({max_vel_, w_target_accel, w_max_kinematic});
            w_ideal = std::max(w_ideal, min_sustainable_vel_);
            
            w_cmd_ = std::clamp(w_ideal, 0.0, max_vel_);

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0; 
            msg.angular.z = w_cmd_ * direction;
            cmd_pub_->publish(msg);
        }
        
        // [UPDATE] Settling Phase: Active P-Control to hold angle
        if (state_ == SETTLING) {
            if ((now - settling_start_time_).seconds() > 0.5) {
                state_ = DONE;
                RCLCPP_INFO(this->get_logger(), "Settling complete. Final Traveled: %.4f rad.", yaw_traveled_);
                return;
            }

            // Simple P-Controller to counter drift/inertia
            double error = normalize_angle(target_angle_ - yaw_traveled_);
            double p_gain = 2.0; // Responsive enough for small adjustments
            double w_hold = std::clamp(p_gain * error, -max_vel_, max_vel_);

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = w_hold;
            cmd_pub_->publish(msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotationDriver>());
    rclcpp::shutdown();
    return 0;
}