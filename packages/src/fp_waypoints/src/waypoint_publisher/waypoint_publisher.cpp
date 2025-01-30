#include "fp_waypoints/waypoint_publisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/logging.hpp>

WaypointPublisher::WaypointPublisher() : Node("waypoint_publisher") {
  RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node Started");

  // waypoint publisher
  waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "robot_waypoints", rclcpp::QoS(10));

  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  this->declare_parameter<int>("number_of_waypoints", 5);
  this->get_parameter<int>("number_of_waypoints", number_of_waypoints_);

  this->declare_parameter<int>("waypoint_cutoff", 2);
  this->get_parameter<int>("waypoint_cutoff", waypoint_cutoff_);

  // print waypoint file
  RCLCPP_INFO(this->get_logger(), "Waypoint file: %s", waypoint_file_.c_str());

  std::ifstream file(waypoint_file_, std::ifstream::in);
  std::string line;
  if (file.is_open()) {
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string x, y, yaw;
      iss >> x >> y >> yaw;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = std::stof(x);
      pose.pose.position.y = std::stof(y);
      pose.pose.position.z = 0;

      double heading =
          std::stod(yaw) * M_PI / 180.0;  // Convert degrees to radians

      tf2::Quaternion quat;
      quat.setRPY(0, 0, heading);
      pose.pose.orientation = tf2::toMsg(quat);

      poses_.push_back(pose);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file");
    return;
  }
  file.close();

  // to start from zero at first iteration
  current_pose_index_ = -number_of_waypoints_;

  // print poses
  RCLCPP_INFO(this->get_logger(), "Poses:");
  for (auto pose : poses_) {
    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::fromMsg(pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f",
                pose.pose.position.x, pose.pose.position.y,
                yaw * 180.0 / M_PI);  // Convert radians to degrees
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                   [this]() { timer_callback(); });

  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");
}

void WaypointPublisher::timer_callback() {
  this->get_parameter<int>("number_of_waypoints", number_of_waypoints_);
  this->get_parameter("waypoint_cutoff", waypoint_cutoff_);
  publish_waypoints();
  timer_->cancel();
}

void WaypointPublisher::publish_waypoints() {
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints");

  current_pose_index_ = (current_pose_index_ + number_of_waypoints_ -
                         remaining_number_of_poses_) %
                        poses_.size();

  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_;
       i < current_pose_index_ + number_of_waypoints_; i++) {
    goal_msg.poses.push_back(poses_[i % poses_.size()]);
  }

  // publish poses to rviz
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  // manually set the pauses
  for (uint i = 0; i < goal_msg.poses.size(); i++) {
    pose_array.poses.push_back(goal_msg.poses[i].pose);
  }
  waypoint_publisher_->publish(pose_array);

  auto send_goal_options =
      rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](GoalHandleNavigateThroughPoses::SharedPtr goal_handle) {
        this->goal_response_callback(goal_handle);
      };
  send_goal_options.feedback_callback =
      [this](GoalHandleNavigateThroughPoses::SharedPtr goal_handle,
             const std::shared_ptr<const NavigateThroughPoses::Feedback>
                 feedback) { this->feedback_callback(goal_handle, feedback); };
  send_goal_options.result_callback =
      [this](const GoalHandleNavigateThroughPoses::WrappedResult& result) {
        this->result_callback(result);
      };

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Sent goal");
  remaining_number_of_poses_ = number_of_waypoints_;
}

void WaypointPublisher::goal_response_callback(
    GoalHandleNavigateThroughPoses::SharedPtr future) {
  auto goal_handle = future.get();
  RCLCPP_INFO(this->get_logger(), "Goal response received");
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    publish_waypoints();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void WaypointPublisher::feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Received feedback: %i",
              feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;

  this->get_parameter("waypoint_cutoff", waypoint_cutoff_);
  if (remaining_number_of_poses_ < waypoint_cutoff_) {
    timer_->reset();
  }
}

void WaypointPublisher::result_callback(
    const GoalHandleNavigateThroughPoses::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      publish_waypoints();
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      publish_waypoints();
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation finished");
  timer_->reset();
}
