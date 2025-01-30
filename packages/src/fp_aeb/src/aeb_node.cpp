#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>

class AEBNode : public rclcpp::Node {
public:
    AEBNode() : Node("aeb_node"), vehicle_speed_(0.0), ttc_threshold_(0.5) {
        initializeSubscribers();
        initializePublishers();
        initializeTimer();
    }

private:
    // Constants
    static constexpr int QUEUE_SIZE = 10;
    static constexpr int TIMER_MS = 1;
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan latest_scan_;
    double vehicle_speed_;
    const double ttc_threshold_;

    void initializeSubscribers() {
        auto scan_callback = std::bind(&AEBNode::scanCallback, this, std::placeholders::_1);
        auto odom_callback = std::bind(&AEBNode::odomCallback, this, std::placeholders::_1);
        
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/robot/scan", QUEUE_SIZE, scan_callback);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/robot/odom", QUEUE_SIZE, odom_callback);
    }

    void initializePublishers() {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
    }

    void initializeTimer() {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(TIMER_MS),
            std::bind(&AEBNode::collisionDetectionCallback, this)
        );
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = *msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        vehicle_speed_ = msg->twist.twist.linear.x;
    }

    double calculateTTC(double range, double angle) const {
        double range_rate = vehicle_speed_ * std::cos(angle);
        return (range_rate > 0) ? range / range_rate : std::numeric_limits<double>::infinity();
    }

    void applyEmergencyBrake() {
        RCLCPP_WARN(get_logger(), "⚠️ EMERGENCY STOP: Obstacle detected in critical zone!");
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_INFO(get_logger(), "Safety system engaged - Vehicle stopping");
    }

    void collisionDetectionCallback() {
        if (latest_scan_.ranges.empty()) return;

        double min_ttc = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            double ttc = calculateTTC(latest_scan_.ranges[i], angle);
            min_ttc = std::min(min_ttc, ttc);
        }

        if (min_ttc < ttc_threshold_) {
            applyEmergencyBrake();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AEBNode>());
    rclcpp::shutdown();
    return 0;
}