#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  linear_speed_ = 0.5;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,  [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      robot_odom_ = msg;
    });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  if (targetReached()) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Target reached.");
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);

  cmd_vel_pub_->publish(cmd_vel);
}

bool ControlNode::targetReached() {
  double goal_dist = computeDistance(
    current_path_->poses.back().pose.position, robot_odom_->pose.pose.position);
  return goal_dist < goal_tolerance;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {

  if (!current_path_ || current_path_->poses.empty()) {
    return std::nullopt;
  }

  auto robot_pos = robot_odom_->pose.pose.position;

  for (const auto& pose : current_path_->poses) {
    double distance = computeDistance(pose.pose.position, robot_pos);
    if (distance >= lookahead_distance) {
      return pose;
    }
  }

  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &point) {
  geometry_msgs::msg::Twist cmd_vel;

  auto robot_pos = robot_odom_->pose.pose.position;
  auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
  auto point_pos = point.pose.position;

  double dx = point_pos.x - robot_pos.x;
  double dy = point_pos.y - robot_pos.y;

  double yp = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  double curvature = (2 * yp) / (lookahead_distance * lookahead_distance);

  double angular_vel = linear_speed_ * curvature;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = angular_vel;

  return cmd_vel; 
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}