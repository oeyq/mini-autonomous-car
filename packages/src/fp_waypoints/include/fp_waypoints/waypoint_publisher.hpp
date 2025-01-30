#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class WaypointPublisher : public rclcpp::Node {
 public:
  WaypointPublisher();

 private:
  // create a timer
  rclcpp::TimerBase::SharedPtr timer_;

  // create a timer callback
  void timer_callback();

  // create a function to publish the waypoints
  void publish_waypoints();

  // create a client for the action
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
      client_ptr_;

  // create a goal response callback
  void goal_response_callback(GoalHandleNavigateThroughPoses::SharedPtr future);

  // create a feedback callback
  void feedback_callback(
      GoalHandleNavigateThroughPoses::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);

  // create a result callback
  void result_callback(
      const GoalHandleNavigateThroughPoses::WrappedResult& result);

  // create a vector of poses
  std::vector<geometry_msgs::msg::PoseStamped> poses_;

  // define waypoint file path
  std::string waypoint_file_;

  // define the current pose index
  int current_pose_index_ = 0;

  // define the remaining number of poses
  int remaining_number_of_poses_ = 0;

  // number of waypoints
  int number_of_waypoints_ = 5;

  // waypoint cutoff -> number of waypoints remaining before updating the path
  int waypoint_cutoff_ = 2;

  // publisher for rviz to visualize the waypoints
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      waypoint_publisher_;
};
