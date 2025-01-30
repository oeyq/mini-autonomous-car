#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include <cmath>
#include <chrono>
#include <limits> // for std::numeric_limits

class RaceStatisticsNode : public rclcpp::Node
{
public:
    RaceStatisticsNode()
    : Node("race_statistics"),
      laps_completed_(0),
      last_cross_time_(0.0),
      finish_line_x_(2.681731),
      first_cross_(true),
      last_robot_x_(std::numeric_limits<double>::quiet_NaN()),
      last_robot_y_(std::numeric_limits<double>::quiet_NaN()),
      current_speed_(0.0),
      last_lap_time_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Race Statistics Node started.");

        // Subscribe to /robot/odom (to track and print speed)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odom", 10,
            std::bind(&RaceStatisticsNode::odomCallback, this, std::placeholders::_1)
        );

        // Subscribe to /model_states (to track position and detect laps)
        model_states_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&RaceStatisticsNode::modelStatesCallback, this, std::placeholders::_1)
        );
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 1. Calculate current speed
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_speed_ = std::sqrt(vx * vx + vy * vy);

        // 2. Print a single line with speed, number of laps, and last lap time
        //    (If no laps completed yet, last_lap_time_ is 0.0)
        RCLCPP_INFO(
            this->get_logger(),
            "Speed: %.2f m/s | Lap: %d | Time: %.2f s",
            current_speed_,
            laps_completed_,
            laps_completed_ == 0 ? 0.0 : last_lap_time_
        );
    }

    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Find "robot" position in ModelStates
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "robot")
            {
                double robot_x = msg->pose[i].position.x;
                double robot_y = msg->pose[i].position.y;
                double current_time = this->now().seconds();

                // If first time callback, store pose/time and return
                if (std::isnan(last_robot_x_))
                {
                    last_robot_x_ = robot_x;
                    last_robot_y_ = robot_y;
                    last_cross_time_ = current_time;
                    return;
                }

                // Check if we cross the line:
                //    - last X < finish_line_x_  
                //    - current X >= finish_line_x_  
                //    - y in [-1.0,  1.0]
                if ((last_robot_x_ < finish_line_x_) &&
                    (robot_x >= finish_line_x_) &&
                    (robot_y >= -1.0 && robot_y <= 1.0))
                {
                    if (first_cross_)
                    {
                        // First crossing => start timing, but don't increment laps
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Crossed finish line for the first time. Starting lap timing now."
                        );
                        first_cross_ = false;
                        last_cross_time_ = current_time;
                    }
                    else
                    {
                        // Completed a lap
                        laps_completed_++;
                        last_lap_time_ = current_time - last_cross_time_;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Lap %d completed in %.2f seconds.",
                            laps_completed_,
                            last_lap_time_
                        );

                        // Update the last cross time
                        last_cross_time_ = current_time;
                    }
                }

                // Update last-known position
                last_robot_x_ = robot_x;
                last_robot_y_ = robot_y;
                break;
            }
        }
    }

    // --- Laps / Race
    int laps_completed_;
    double last_cross_time_;
    double finish_line_x_;
    bool first_cross_;

    // --- Pose from last callback
    double last_robot_x_;
    double last_robot_y_;

    // --- Speed and Lap Time
    double current_speed_;
    double last_lap_time_;

    // --- Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaceStatisticsNode>());
    rclcpp::shutdown();
    return 0;
}
