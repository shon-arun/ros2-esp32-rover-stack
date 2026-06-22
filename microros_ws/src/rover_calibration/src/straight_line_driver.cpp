#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>

class StraightLineDriver : public rclcpp::Node {
public:
    StraightLineDriver() : Node("straight_line_driver"), state_(WAITING_FOR_ODOM), v_cmd_(0.0), pose_initialized_(false), stuck_cycles_(0) {
        // --- Parameters ---
        this->declare_parameter("target_distance", 1.0);       
        this->declare_parameter("max_velocity", 0.2);          
        this->declare_parameter("max_acceleration", 0.15);     
        this->declare_parameter("kp_yaw", 1.5);                
        this->declare_parameter("tolerance", 0.005);           
        this->declare_parameter("min_sustainable_vel", 0.08);  

        target_distance_ = this->get_parameter("target_distance").as_double();
        max_vel_ = this->get_parameter("max_velocity").as_double();
        max_accel_ = this->get_parameter("max_acceleration").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        min_sustainable_vel_ = this->get_parameter("min_sustainable_vel").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&StraightLineDriver::odom_callback, this, std::placeholders::_1));

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

    double euler_from_quaternion(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

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
        if (state_ == WAITING_FOR_ODOM) return;

        // Ensure we cleanly hold the 0 state without killing the node asynchronously 
        if (state_ == DONE) {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_pub_->publish(stop_msg);
            return;
        }

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
                state_ = DONE;
                RCLCPP_INFO(this->get_logger(), "Target perfectly reached. Traveled: %.4fm. (Press Ctrl+C to exit)", distance_traveled);
                return;
            }

            // 2. Kinematic Velocity Profiling 
            double v_max_kinematic = std::sqrt(2.0 * max_accel_ * std::max(0.0, distance_remaining));
            double v_target_accel = v_cmd_ + max_accel_ * dt_;
            
            double v_ideal = std::min({max_vel_, v_target_accel, v_max_kinematic});
            v_ideal = std::max(v_ideal, 0.0); 

            // --- FIX: Jump-start acceleration to overcome static friction ---
            if (v_ideal > 0.0 && v_ideal < min_sustainable_vel_ && v_max_kinematic >= min_sustainable_vel_) {
                v_ideal = min_sustainable_vel_;
            }

            // 3. Hardware Cutoff & Settling Watchdog (Now only triggers on deceleration phase)
            if (v_ideal < min_sustainable_vel_) {
                v_cmd_ = 0.0; // Cut power, let momentum coast it in
                stuck_cycles_++;
                
                if (stuck_cycles_ > 50) {
                    RCLCPP_WARN(this->get_logger(), "Settled short at %.3fm due to friction. Terminating.", distance_traveled);
                    state_ = DONE;
                    return;
                }
            } else {
                v_cmd_ = v_ideal;
                stuck_cycles_ = 0; 
            }

            // 4. Active Heading Correction
            double yaw_error = normalize_angle(start_yaw_ - current_yaw_);
            double angular_cmd = kp_yaw_ * yaw_error;
            
            // --- FIX: Clamp extreme angular corrections to prevent violent spins ---
            angular_cmd = std::clamp(angular_cmd, -1.0, 1.0); 

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