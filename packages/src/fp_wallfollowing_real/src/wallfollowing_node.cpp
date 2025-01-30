#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// Automatic Emergency Braking Node - Controls robot movement along walls
class AEBNode : public rclcpp::Node {
public:
    AEBNode();

private:
    // Configuration parameters for the controller
    struct Parameters {
        double desired_distance{0.4};    // Target distance from wall
        double kp{1.0};                  // Proportional gain
        double ki{0.005};                // Integral gain
        double kd{0.001};                // Derivative gain
        double lookahead_distance{0.5};  // Distance to look ahead for planning
        double min_dist_right{1.0};      // Minimum right wall distance
        double min_dist_left{0.8};       // Minimum left wall distance
        double path_length{1.0};         // Robot path length
        double max_angular_velocity{0.6}; // Maximum turning speed
    };

    // Robot state variables
    struct State {
        double prev_error{0.0};          // Previous distance error
        double integral{0.0};            // Integral term for PID
        double last_time{0.0};           // Last control loop time
        sensor_msgs::msg::LaserScan::SharedPtr lidar_scan;  // Latest scan data
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void controlLoop();
    double getDistanceAtAngle(double angle) const;
    double calculateSpeed(double angular_velocity) const;
    
    // Wall distance measurements
    struct WallMeasurements {
        double alpha;                // Wall angle
        double current_distance;     // Current distance to wall
        double predicted_distance;   // Predicted distance after movement
    };
    
    WallMeasurements calculateWallDistances(double dist_a, double dist_b, 
                                          double ang_a, double ang_b) const;

    // ROS2 communication handlers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Parameters params_;
    State state_;
};

// Initialize ROS2 node and setup communications
AEBNode::AEBNode() : Node("aeb_node") {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/robot/scan", 1,
        std::bind(&AEBNode::scanCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
    timer_ = this->create_wall_timer(10ms, std::bind(&AEBNode::controlLoop, this));
}

// Store latest LiDAR scan
void AEBNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    state_.lidar_scan = msg;
}

// Get distance measurement at specific angle from LiDAR scan
double AEBNode::getDistanceAtAngle(double angle) const {
    if (!state_.lidar_scan) return std::numeric_limits<double>::infinity();
    
    int index = (angle - state_.lidar_scan->angle_min) / state_.lidar_scan->angle_increment;
    if (index >= 0 && index < static_cast<int>(state_.lidar_scan->ranges.size())) {
        return state_.lidar_scan->ranges[index];
    }
    return std::numeric_limits<double>::infinity();
}

// Calculate wall distances and orientation
AEBNode::WallMeasurements AEBNode::calculateWallDistances(
    double dist_a, double dist_b, double ang_a, double ang_b) const {
    
    WallMeasurements measurements;
    // Calculate wall angle
    measurements.alpha = std::atan((dist_a * std::cos(ang_b - ang_a) - dist_b) / 
                                 (dist_a * std::sin(ang_b - ang_a)));
    
    // Calculate current and predicted distances
    measurements.current_distance = dist_b * std::cos(measurements.alpha);
    measurements.predicted_distance = measurements.current_distance + 
                                    params_.path_length * std::sin(measurements.alpha);
    
    return measurements;
}

// Calculate robot speed based on turning rate
double AEBNode::calculateSpeed(double angular_velocity) const {
    const double abs_velocity = std::fabs(angular_velocity);
    if (abs_velocity > 0.2) return 0.75;      // Sharp turn
    if (abs_velocity > 0.1) return 1.0;      // Medium turn
    return 1.25;                              // Straight/gentle turn
}

// Main control loop
void AEBNode::controlLoop() {
    if (!state_.lidar_scan) return;

    // Get distances at 45° and 90°
    const double ang_a = 45.0 * M_PI / 180.0;
    const double ang_b = 90.0 * M_PI / 180.0;
    
    const double dist_a = getDistanceAtAngle(ang_a);
    const double dist_b = getDistanceAtAngle(ang_b);

    // Calculate wall distances and orientation
    auto measurements = calculateWallDistances(dist_a, dist_b, ang_a, ang_b);
    
    // Calculate error from desired distance
    const double error = params_.min_dist_left - measurements.predicted_distance;
    
    // Calculate time delta for PID
    const auto current_time = std::chrono::steady_clock::now();
    const double delta_time = std::chrono::duration<double>(
        current_time - std::chrono::steady_clock::time_point()).count();
    
    // Update integral term
    state_.integral += state_.prev_error * delta_time;
    
    // Calculate PD control (Integral term disabled)
    double angular_velocity = -(params_.kp * error + 
                              params_.kd * (error - state_.prev_error) / delta_time);
    
    // Limit angular velocity
    angular_velocity = std::clamp(angular_velocity, 
                                -params_.max_angular_velocity,
                                params_.max_angular_velocity);

    state_.prev_error = error;

    // Create and publish velocity command
    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = calculateSpeed(angular_velocity);
    cmd_vel.angular.z = angular_velocity;
    cmd_vel_pub_->publish(cmd_vel);

    // Log current state
    RCLCPP_INFO(this->get_logger(),
                "Distance: %.2f, Error: %.2f, Speed: %.2f, Angle: %.2f",
                measurements.predicted_distance, error, cmd_vel.linear.x, angular_velocity);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AEBNode>());
    rclcpp::shutdown();
    return 0;
}
