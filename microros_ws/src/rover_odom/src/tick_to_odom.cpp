#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <deque>

class TickToOdom : public rclcpp::Node
{
public:
    TickToOdom() : Node("tick_to_odom"), x_(0.0), y_(0.0), th_(0.0), first_reading_(true)
    {
        last_time_ = this->get_clock()->now();

        // --- HARDCODED CHASSIS CONSTANTS ---
        wheel_radius_ = 0.0325; // 65mm standard yellow wheel
        track_width_ = 0.20;    // ~20cm track width for acrylic kit
        // ticks_per_rev_ = 20.0;  // 20 slots
        ticks_per_rev_ = 40.0;  // 20 slots

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        tick_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/wheel_ticks", rclcpp::SensorDataQoS(),
            std::bind(&TickToOdom::tick_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Tick-to-Odometry Node Initialized (TF Broadcaster Disabled for EKF).");
    }

private:
    void tick_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;

        if (first_reading_) {
            last_ticks_FL_ = msg->data[0];
            last_ticks_RL_ = msg->data[1];
            last_ticks_FR_ = msg->data[2];
            last_ticks_RR_ = msg->data[3];
            last_time_ = this->get_clock()->now();
            first_reading_ = false;
            return;
        }

        // 1. Calculate Delta Ticks for each independent wheel
        double delta_FL = msg->data[0] - last_ticks_FL_;
        double delta_RL = msg->data[1] - last_ticks_RL_;
        double delta_FR = msg->data[2] - last_ticks_FR_;
        double delta_RR = msg->data[3] - last_ticks_RR_;

        // 2. Convert Ticks to Physical Distance (Meters)
        double dist_per_tick = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;
        double d_FL = delta_FL * dist_per_tick;
        double d_RL = delta_RL * dist_per_tick;
        double d_FR = delta_FR * dist_per_tick;
        double d_RR = delta_RR * dist_per_tick;

        // 3. Average the physical distances for the left and right sides
        double d_left = (d_FL + d_RL) / 2.0;
        double d_right = (d_FR + d_RR) / 2.0;

        // 4. Calculate Center Translation and Rotation
        double d_center = (d_left + d_right) / 2.0;
        double d_theta = (d_right - d_left) / track_width_;

        // 5. Integrate into Global Position
        x_ += d_center * cos(th_ + (d_theta / 2.0));
        y_ += d_center * sin(th_ + (d_theta / 2.0));
        th_ += d_theta;

        // 6. Calculate Dynamic Time Delta (dt)
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();

        if (dt <= 0.0) {
            last_time_ = now;
            return; 
        }

        // Calculate raw velocities and apply Moving Average Filter
        // (UPDATED: Using velocity of averages to mitigate network jitter)
        dx_history_.push_back(d_center);
        dz_history_.push_back(d_theta);
        dt_history_.push_back(dt);

        if (dt_history_.size() > filter_window_size_) {
            dx_history_.pop_front();
            dz_history_.pop_front();
            dt_history_.pop_front();
        }

        double total_dx = 0.0;
        double total_dz = 0.0;
        double total_dt = 0.0;
        for (double dx : dx_history_) total_dx += dx;
        for (double dz : dz_history_) total_dz += dz;
        for (double t : dt_history_) total_dt += t;
        
        double filtered_v_x = (total_dt > 0.0) ? (total_dx / total_dt) : 0.0;
        double filtered_v_z = (total_dt > 0.0) ? (total_dz / total_dt) : 0.0;

        // 7. Publish Odometry Message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Calculate and populate rotation quaternion locally for the odom topic
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.pose.covariance[0] = 0.1;
        odom.pose.covariance[7] = 0.1;
        odom.pose.covariance[35] = 0.1;

        odom.twist.covariance[0] = 0.01;
        odom.twist.covariance[7] = 1e-9;
        odom.twist.covariance[35] = 0.5;

        odom.twist.twist.linear.x = filtered_v_x;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = filtered_v_z;

        odom_pub_->publish(odom);

        // 8. Update state for the next loop
        last_ticks_FL_ = msg->data[0];
        last_ticks_RL_ = msg->data[1];
        last_ticks_FR_ = msg->data[2];
        last_ticks_RR_ = msg->data[3];
        last_time_ = now;
    }

    double wheel_radius_;
    double track_width_;
    double ticks_per_rev_;

    double x_, y_, th_;
    
    double last_ticks_FL_, last_ticks_RL_;
    double last_ticks_FR_, last_ticks_RR_;
    
    bool first_reading_;
    rclcpp::Time last_time_;

    std::deque<double> dx_history_;
    std::deque<double> dz_history_;
    std::deque<double> dt_history_;
    const size_t filter_window_size_ = 5;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tick_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TickToOdom>());
    rclcpp::shutdown();
    return 0;
}