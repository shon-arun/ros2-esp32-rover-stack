#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

class RotationDriver : public rclcpp::Node {
public:
    RotationDriver() : Node("rotation_driver"), state_(WAITING_FOR_ODOM), w_cmd_(0.0), pose_initialized_(false), yaw_traveled_(0.0) {
        // --- Parameters ---
        // target_angle is in Radians. Default: 1.5708 rad (90 degrees)
        this->declare_parameter("target_angle", 1.5708);       
        this->declare_parameter("max_angular_velocity", 1.0);          
        this->declare_parameter("max_angular_acceleration", 0.5);     
        this->declare_parameter("tolerance", 0.02);           
        this->declare_parameter("min_sustainable_angular_vel", 0.15);  

        target_angle_ = this->get_parameter("target_angle").as_double();
        max_vel_ = this->get_parameter("max_angular_velocity").as_double();
        max_accel_ = this->get_parameter("max_angular_acceleration").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        min_sustainable_vel_ = this->get_parameter("min_sustainable_angular_vel").as_double();

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
    enum State { WAITING_FOR_ODOM, INIT, ROTATING, DONE };
    State state_;
    bool pose_initialized_;

    double current_yaw_;
    double yaw_traveled_; 
    double w_cmd_; // Angular velocity command

    double target_angle_, max_vel_, max_accel_, tolerance_, min_sustainable_vel_;
    
    rclcpp::Time last_time_;
    rclcpp::Time last_odom_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Fast quaternion to yaw calculation
    inline double euler_from_quaternion(double x, double y, double z, double w) const {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    // O(1) angle normalization using std::remainder (-pi to pi)
    inline double normalize_angle(double angle) const {
        return std::remainder(angle, 2.0 * M_PI);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = this->get_clock()->now();

        auto q = msg->pose.pose.orientation;
        double new_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w);

        if (!pose_initialized_) {
            current_yaw_ = new_yaw;
            pose_initialized_ = true;
            state_ = INIT;
        } else {
            // Calculate the shortest delta to handle pi/-pi wrap-around
            double delta_yaw = normalize_angle(new_yaw - current_yaw_);
            current_yaw_ = new_yaw;
            
            // Accumulate unwrapped travel distance
            if (state_ == ROTATING) {
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
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Ensure ROS transmits
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
            RCLCPP_INFO(this->get_logger(), "Starting Rotation: Target Angle = %.2f rad (%.1f deg)", 
                        target_angle_, target_angle_ * 180.0 / M_PI);
        }

        if (state_ == ROTATING) {
            double direction = (target_angle_ >= 0.0) ? 1.0 : -1.0;

            // Robust crossing check depending on rotational direction
            bool reached_target = (direction > 0.0) 
                ? (yaw_traveled_ >= target_angle_ - tolerance_)
                : (yaw_traveled_ <= target_angle_ + tolerance_);

            if (reached_target) {
                state_ = DONE;
                RCLCPP_INFO(this->get_logger(), "Rotation target reached. Traveled: %.4f rad. Shutting down.", yaw_traveled_);
                return;
            }

            double angle_remaining = std::abs(target_angle_ - yaw_traveled_);

            // Kinematic Velocity Profiling (Slew rate limiter & Deceleration curve)
            double w_max_kinematic = std::sqrt(2.0 * max_accel_ * angle_remaining);
            double w_target_accel = w_cmd_ + (max_accel_ * dt);
            
            double w_ideal = std::min({max_vel_, w_target_accel, w_max_kinematic});
            
            // Maintain minimum sustainable speed to prevent static friction stalls
            w_ideal = std::max(w_ideal, min_sustainable_vel_);
            
            w_cmd_ = std::clamp(w_ideal, 0.0, max_vel_);

            // Publish Command
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0; 
            msg.angular.z = w_cmd_ * direction;
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