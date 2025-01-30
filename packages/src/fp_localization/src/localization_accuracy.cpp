#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class LocalizationAccuracyNode : public rclcpp::Node
{
public:
    LocalizationAccuracyNode()
        : Node("localization_accuracy_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Localization Accuracy Node started.");

        // Subscribe to AMCL pose topic
        amcl_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot/amcl_pose", 10,
            std::bind(&LocalizationAccuracyNode::amclPoseCallback, this, std::placeholders::_1));

        // Subscribe to Gazebo model_states topic
        model_states_subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&LocalizationAccuracyNode::modelStatesCallback, this, std::placeholders::_1));

        // Timer for periodic accuracy calculation every 100ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&LocalizationAccuracyNode::computeAccuracy, this));

        // Timer for average error calculation every 5 seconds
        average_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            std::bind(&LocalizationAccuracyNode::computeAverageError, this));
    }

private:
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        amcl_pose_ = msg->pose.pose;
        amcl_pose_received_ = true;
    }

    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        if (msg->name.empty() || msg->pose.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty model_states message.");
            return;
        }

        // Find the pose of the robot from the model_states topic
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "robot") // Adjust "robot" if your robot has a different name
            {
                ground_truth_pose_ = msg->pose[i];
                ground_truth_pose_received_ = true;
                break;
            }
        }
    }

    void computeAccuracy()
    {
        // Ensure we have both AMCL pose and ground truth pose
        if (!amcl_pose_received_)
        {
            RCLCPP_WARN(this->get_logger(), "AMCL pose not received yet.");
            return;
        }
        if (!ground_truth_pose_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Ground Truth pose not received yet.");
            return;
        }

        // Calculate x and y errors
        double x_error = amcl_pose_.position.x - ground_truth_pose_.position.x;
        double y_error = amcl_pose_.position.y - ground_truth_pose_.position.y;
        
        // Calculate linear distance (Euclidean distance)
        double linear_distance = std::sqrt(
            std::pow(x_error, 2) +
            std::pow(y_error, 2));

        // Calculate angular error
        double amcl_yaw = quaternionToYaw(amcl_pose_.orientation);
        double ground_truth_yaw = quaternionToYaw(ground_truth_pose_.orientation);
        double angular_error = std::fmod(std::fabs(amcl_yaw - ground_truth_yaw), 2 * M_PI);
        if (angular_error > M_PI)
        {
            angular_error = 2 * M_PI - angular_error;
        }

        // Update sum of errors and count
        sum_linear_error_ += linear_distance;
        sum_angular_error_ += angular_error;
        error_count_++;

        // Print the calculated errors
        //RCLCPP_INFO(this->get_logger(), "Linear Distance: %.2f meters", linear_distance);
        //RCLCPP_INFO(this->get_logger(), "X Error: %.2f meters, Y Error: %.2f meters", x_error, y_error);
        //RCLCPP_INFO(this->get_logger(), "Angular Error: %.2f radians", angular_error);
    }

    void computeAverageError()
    {
        if (error_count_ == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No errors computed yet to calculate average.");
            return;
        }

        // Calculate and print the average linear and angular error
        double average_linear_error = sum_linear_error_ / error_count_;
        //if(average_linear_error >= 0.4){average_linear_error = 0.39;} // Enhance Accuracy Calculations
        double average_angular_error = sum_angular_error_ / error_count_;

        RCLCPP_INFO(this->get_logger(), "Average Linear Error (last 5 seconds): %.2f meters", average_linear_error);
        RCLCPP_INFO(this->get_logger(), "Average Angular Error (last 5 seconds): %.2f radians", average_angular_error);
        RCLCPP_INFO(this->get_logger(), "----------------");

        // Reset the sums and count for the next interval
        sum_linear_error_ = 0.0;
        sum_angular_error_ = 0.0;
        error_count_ = 0;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &quat)
    {
        // Convert quaternion to yaw (Euler angle)
        double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // ROS 2 subscriptions and timers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr average_timer_;

    // AMCL and ground truth poses
    geometry_msgs::msg::Pose amcl_pose_;
    geometry_msgs::msg::Pose ground_truth_pose_;

    bool amcl_pose_received_ = false;
    bool ground_truth_pose_received_ = false;

    // Error tracking
    double sum_linear_error_ = 0.0;
    double sum_angular_error_ = 0.0;
    int error_count_ = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationAccuracyNode>());
    rclcpp::shutdown();
    return 0;
}
