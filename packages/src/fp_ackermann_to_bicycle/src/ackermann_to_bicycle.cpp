#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class AckermannToBicyleNode : public rclcpp::Node
{
public:
    AckermannToBicyleNode() : Node("ackermann_to_bicycle")
    {
        RCLCPP_INFO(this->get_logger(), "Converter from Ackermann to Bicycle started.");

        cmd_vel_nav_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "robot/cmd_vel_nav", 10, std::bind(&AckermannToBicyleNode::cmdVelNavCallback, this, std::placeholders::_1));

        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    }

private:
    void cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
         double linear_velocity = msg->linear.x;
         double angular_velocity = msg->angular.z;

         double steering_angle = 0.0;
         auto twist_msg = geometry_msgs::msg::Twist();

         steering_angle = std::atan((angular_velocity * wheelbase) / linear_velocity);

         twist_msg.linear.x = linear_velocity;
         twist_msg.angular.z = steering_angle;
         cmd_vel_publisher->publish(twist_msg);

    }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
        const double wheelbase = 0.32;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToBicyleNode>());
    rclcpp::shutdown();
    return 0;
}