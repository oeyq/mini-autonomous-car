#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
            "model_states", 10,
            std::bind(&LocalizationAccuracyNode::modelStatesCallback, this, std::placeholders::_1));

        // Timer for periodic accuracy calculation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&LocalizationAccuracyNode::computeAccuracy, this));
    }

private:
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        amcl_pose_ = msg->pose.pose;
        amcl_pose_received_ = true;
    }

    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Get the ground truth pose from Gazebo
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "robot")  
            {
                ground_truth_pose_ = msg->pose[i];
                ground_truth_pose_received_ = true;
                break;
            }
        }
    }

    void computeAccuracy()
    {
        if (!amcl_pose_received_ || !ground_truth_pose_received_)
        {
            return;
        }

        try {
            // Get the transform from map to base_link
            // geometry_msgs::msg::TransformStamped transform_stamped = 
            //     tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // Calculate linear errors
            double x_error = amcl_pose_.position.x - ground_truth_pose_.position.x;
            double y_error = amcl_pose_.position.y - ground_truth_pose_.position.y;
            
            // Calculate linear distance (Euclidean distance)
            double linear_distance = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2));

            // Calculate angular error
            double amcl_yaw = quaternionToYaw(amcl_pose_.orientation);
            double true_yaw = quaternionToYaw(ground_truth_pose_.orientation);
            
            // Normalize angular error to [-π, π]
            double angular_error = std::atan2(std::sin(amcl_yaw - true_yaw), 
                                            std::cos(amcl_yaw - true_yaw));

            // Log the results
            RCLCPP_INFO(this->get_logger(), 
                "Localization Errors:\n"
                "  Linear Distance: %.2f meters\n"
                "  X Error: %.2f meters\n"
                "  Y Error: %.2f meters\n"
                "  Angular Error: %.2f radians",
                linear_distance, x_error, y_error, angular_error);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &quat)
    {
        // Extract yaw from quaternion
        double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // TF2 objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Pose storage
    geometry_msgs::msg::Pose amcl_pose_;
    geometry_msgs::msg::Pose ground_truth_pose_;
    bool amcl_pose_received_ = false;
    bool ground_truth_pose_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationAccuracyNode>());
    rclcpp::shutdown();
    return 0;
}