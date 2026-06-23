#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <cmath>
#include <algorithm>
#include <chrono>

class SquareDriver : public rclcpp::Node {
public:
    SquareDriver() : Node("square_driver"), state_(WAITING_FOR_ODOM), pose_initialized_(false), 
                     side_count_(0), v_cmd_(0.0), w_cmd_(0.0), yaw_traveled_(0.0) {
        
        // --- Straight Line Parameters ---
        this->declare_parameter("target_distance", 1.0);       
        this->declare_parameter("max_velocity", 0.2);          
        this->declare_parameter("max_acceleration", 0.2);     
        this->declare_parameter("kp_yaw", 1.5);                
        this->declare_parameter("max_yaw_correction_vel", 1.0); 
        this->declare_parameter("linear_tolerance", 0.005);           
        this->declare_parameter("min_sustainable_vel", 0.08);  

        // --- Rotation Parameters ---
        this->declare_parameter("target_angle", 1.5708);
        this->declare_parameter("max_angular_velocity", 5.0);       
        this->declare_parameter("max_angular_acceleration", 2.5);     
        this->declare_parameter("angular_tolerance", 0.15);           
        this->declare_parameter("min_sustainable_angular_vel", 3.0);  

        // Fetch and sanitize parameters
        target_distance_ = std::abs(this->get_parameter("target_distance").as_double());
        max_vel_ = std::abs(this->get_parameter("max_velocity").as_double());
        max_accel_ = std::abs(this->get_parameter("max_acceleration").as_double());
        kp_yaw_ = std::abs(this->get_parameter("kp_yaw").as_double());
        max_yaw_correction_vel_ = std::abs(this->get_parameter("max_yaw_correction_vel").as_double());
        linear_tolerance_ = std::abs(this->get_parameter("linear_tolerance").as_double());
        min_sustainable_vel_ = std::abs(this->get_parameter("min_sustainable_vel").as_double());

        target_angle_ = this->get_parameter("target_angle").as_double();
        max_angular_velocity_ = std::abs(this->get_parameter("max_angular_velocity").as_double());
        max_angular_acceleration_ = std::abs(this->get_parameter("max_angular_acceleration").as_double());
        angular_tolerance_ = std::abs(this->get_parameter("angular_tolerance").as_double());
        min_sustainable_angular_vel_ = std::abs(this->get_parameter("min_sustainable_angular_vel").as_double());

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&SquareDriver::odom_callback, this, std::placeholders::_1));

        // Start clocks
        last_time_ = this->get_clock()->now();
        last_odom_time_ = last_time_;

        // 50 Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SquareDriver::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Professional Square Planner Initialized. Waiting for EKF Odometry...");
    }

private:
    enum State { WAITING_FOR_ODOM, INIT_SIDE, DRIVING, INIT_TURN, ROTATING, SETTLING, DONE };
    State state_;
    bool pose_initialized_;
    int side_count_;

    double current_x_, current_y_, current_yaw_;
    double start_x_, start_y_, start_yaw_;
    
    double v_cmd_;
    double w_cmd_;
    double yaw_traveled_;

    // Combined parameters
    double target_distance_, max_vel_, max_accel_, kp_yaw_, max_yaw_correction_vel_, linear_tolerance_, min_sustainable_vel_;
    double target_angle_, max_angular_velocity_, max_angular_acceleration_, angular_tolerance_, min_sustainable_angular_vel_;
    
    rclcpp::Time last_time_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time settling_start_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // O(1) angle normalization using std::remainder
    inline double normalize_angle(double angle) const {
        static const double TWO_PI = 2.0 * std::acos(-1.0);
        return std::remainder(angle, TWO_PI);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = this->get_clock()->now();
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Utilizing tf2 utility from your configuration directly
        double new_yaw = tf2::getYaw(msg->pose.pose.orientation);

        if (!pose_initialized_) {
            current_yaw_ = new_yaw;
            pose_initialized_ = true;
            state_ = INIT_SIDE; // <--- TYPO FIXED HERE
        } else {
            double delta_yaw = normalize_angle(new_yaw - current_yaw_);
            current_yaw_ = new_yaw;
            
            // Unwrapped accumulation during active turning states
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
        v_cmd_ = 0.0;
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

        // --- 1. INIT SIDE ---
        if (state_ == INIT_SIDE) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_yaw_ = current_yaw_;
            v_cmd_ = 0.0;
            w_cmd_ = 0.0;
            state_ = DRIVING;
            RCLCPP_INFO(this->get_logger(), "Starting Square Side %d/4: Distance = %.2fm", side_count_ + 1, target_distance_);
        }

        // --- 2. DRIVING STRAIGHT ---
        else if (state_ == DRIVING) {
            double dx = current_x_ - start_x_;
            double dy = current_y_ - start_y_;
            
            // Project distance traveled onto starting heading vector
            double distance_traveled = (dx * std::cos(start_yaw_)) + (dy * std::sin(start_yaw_));
            double direction = (target_distance_ >= 0.0) ? 1.0 : -1.0;

            bool reached_target = (direction > 0.0) 
                ? (distance_traveled >= target_distance_ - linear_tolerance_)
                : (distance_traveled <= target_distance_ + linear_tolerance_);

            if (reached_target) {
                state_ = INIT_TURN;
                publish_stop();
                RCLCPP_INFO(this->get_logger(), "Side %d perfectly reached. Initiating turn.", side_count_ + 1);
                return;
            }

            double distance_remaining = std::abs(target_distance_ - distance_traveled);
            double v_max_kinematic = std::sqrt(2.0 * max_accel_ * distance_remaining);
            double v_target_accel = v_cmd_ + (max_accel_ * dt);
            
            double v_ideal = std::min({max_vel_, v_target_accel, v_max_kinematic});
            v_ideal = std::max(v_ideal, min_sustainable_vel_);
            v_cmd_ = std::clamp(v_ideal, 0.0, max_vel_);

            // Active Heading Correction
            double yaw_error = normalize_angle(start_yaw_ - current_yaw_);
            double angular_cmd = std::clamp(kp_yaw_ * yaw_error, -max_yaw_correction_vel_, max_yaw_correction_vel_); 

            geometry_msgs::msg::Twist msg;
            msg.linear.x = v_cmd_ * direction; 
            msg.angular.z = angular_cmd;
            cmd_pub_->publish(msg);
        }

        // --- 3. INIT TURN ---
        else if (state_ == INIT_TURN) {
            yaw_traveled_ = 0.0;
            v_cmd_ = 0.0;
            w_cmd_ = 0.0;
            state_ = ROTATING;
            RCLCPP_INFO(this->get_logger(), "Starting Turn %d: Target Angle = %.2f rad", side_count_ + 1, target_angle_);
        }

        // --- 4. ROTATING ---
        else if (state_ == ROTATING) {
            double direction = (target_angle_ >= 0.0) ? 1.0 : -1.0;

            bool reached_target = (direction > 0.0) 
                ? (yaw_traveled_ >= target_angle_ - angular_tolerance_)
                : (yaw_traveled_ <= target_angle_ + angular_tolerance_);

            if (reached_target) {
                state_ = SETTLING;
                settling_start_time_ = now;
                w_cmd_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Turn boundary crossed. Entering 0.5s settling phase.");
                return;
            }

            double angle_remaining = std::abs(target_angle_ - yaw_traveled_);
            double w_max_kinematic = std::sqrt(2.0 * max_angular_acceleration_ * angle_remaining);
            double w_target_accel = w_cmd_ + (max_angular_acceleration_ * dt);
            
            double w_ideal = std::min({max_angular_velocity_, w_target_accel, w_max_kinematic});
            w_ideal = std::max(w_ideal, min_sustainable_angular_vel_);
            
            w_cmd_ = std::clamp(w_ideal, 0.0, max_angular_velocity_);

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0; 
            msg.angular.z = w_cmd_ * direction;
            cmd_pub_->publish(msg);
        }

        // --- 5. SETTLING ---
        else if (state_ == SETTLING) {
            if ((now - settling_start_time_).seconds() > 0.5) {
                side_count_++;
                if (side_count_ >= 4) {
                    state_ = DONE;
                    RCLCPP_INFO(this->get_logger(), "Square fully completed.");
                } else {
                    state_ = INIT_SIDE;
                    RCLCPP_INFO(this->get_logger(), "Turn Settling complete. Moving to next side.");
                }
                return;
            }

            // P-Controller to counter drift/inertia
            double error = normalize_angle(target_angle_ - yaw_traveled_);
            double p_gain = 2.0; 
            double w_hold = std::clamp(p_gain * error, -max_angular_velocity_, max_angular_velocity_);

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = w_hold;
            cmd_pub_->publish(msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareDriver>());
    rclcpp::shutdown();
    return 0;
}