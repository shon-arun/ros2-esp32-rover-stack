#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class TickToOdom : public rclcpp::Node
{
public:
    TickToOdom() : Node("tick_to_odom"), x_(0.0), y_(0.0), th_(0.0), first_reading_(true)
    {
        // --- HARDCODED CHASSIS CONSTANTS ---
        wheel_radius_ = 0.0325; // 65mm standard yellow wheel
        track_width_ = 0.20;    // ~20cm track width for acrylic kit
        ticks_per_rev_ = 20.0; // 20 slots

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tick_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/wheel_ticks", rclcpp::SensorDataQoS(),
            std::bind(&TickToOdom::tick_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Tick-to-Odometry Node Initialized.");
    }

private:
    void tick_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;

        // 1. Extract and Average the Skid-Steer Encoders [FL, RL, FR, RR]
        double current_left = (msg->data[0] + msg->data[1]) / 2.0;
        double current_right = (msg->data[2] + msg->data[3]) / 2.0;

        if (first_reading_) {
            last_ticks_left_ = current_left;
            last_ticks_right_ = current_right;
            first_reading_ = false;
            return;
        }

        // 2. Calculate Delta Ticks
        double delta_left = current_left - last_ticks_left_;
        double delta_right = current_right - last_ticks_right_;

        // 3. Convert Ticks to Distance (Meters)
        double dist_per_tick = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;
        double d_left = delta_left * dist_per_tick;
        double d_right = delta_right * dist_per_tick;

        // 4. Calculate Center Translation and Rotation
        double d_center = (d_left + d_right) / 2.0;
        double d_theta = (d_right - d_left) / track_width_;

        // 5. Integrate into Global Position
        x_ += d_center * cos(th_ + (d_theta / 2.0));
        y_ += d_center * sin(th_ + (d_theta / 2.0));
        th_ += d_theta;

        // 6. Publish TF Transform (odom -> base_link)
        rclcpp::Time now = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        // 7. Publish Odometry Message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = t.transform.rotation;

        // Pose covariance
        odom.pose.covariance[0] = 0.1;
        odom.pose.covariance[7] = 0.1;
        odom.pose.covariance[35] = 0.1;

        // Twist Covariance (Loosened Anchor)
        odom.twist.covariance[0] = 0.5;  // Linear X velocity variance
        odom.twist.covariance[35] = 0.5; // Angular Z velocity variance

        // Twist (Velocity) - Assuming ~26Hz from ESP32
        double dt = 1.0 / 26.0;
        odom.twist.twist.linear.x = d_center / dt;
        odom.twist.twist.angular.z = d_theta / dt;

        odom_pub_->publish(odom);

        // 8. Update state
        last_ticks_left_ = current_left;
        last_ticks_right_ = current_right;
    }

    double wheel_radius_;
    double track_width_;
    double ticks_per_rev_;

    double x_, y_, th_;
    double last_ticks_left_, last_ticks_right_;
    bool first_reading_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tick_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TickToOdom>());
    rclcpp::shutdown();
    return 0;
}
