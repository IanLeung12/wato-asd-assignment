#include "planner_node.hpp"
#include <cmath> 
#include <vector>
#include <queue>  
#include <unordered_map> 
#include <algorithm>  

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

// Path-finding Algorithm
void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  CellIndex start = worldToMap(robot_pose_.position.x, robot_pose_.position.y, current_map_);
  CellIndex goal = worldToMap(goal_.point.x, goal_.point.y, current_map_);

  // Priority queue for open set
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      nav_msgs::msg::Path path = reconstructPath(came_from, current, current_map_);
      path_pub_->publish(path);
      RCLCPP_INFO(this->get_logger(), "Path found and published!");
      return;
    }

    for (auto neighbor : getNeighbors(current, current_map_)) {
      double tentative_g = g_score[current] + heuristic(current, neighbor);

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f));
      }
    }
  }
  RCLCPP_WARN(this->get_logger(), "No valid path found.");
}


// Converts Real Coordinates into Grid Indices
CellIndex PlannerNode::worldToMap(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map) {
  int x = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(x, y);
}

bool PlannerNode::inBounds(const CellIndex& c, const nav_msgs::msg::OccupancyGrid& map){
  if ((c.x >= 0 && c.x < static_cast<int>(map.info.width)) &&
      (c.y >= 0 && c.y < static_cast<int>(map.info.height))) {
    return 1; 
  } else {
    return 0; 
  }
}

// Checks if a cell is free
bool PlannerNode::isFree(const CellIndex& c, const nav_msgs::msg::OccupancyGrid& map) {
  if (!inBounds(c, map)) {
    return false; // cell is outside the map
  }
  auto index = c.y * map.info.width + c.x;
  int value = map.data[index];
  // Free if 0–49, blocked if >=50, unknown (-1) treated as not free
  return (value >= 0 && value < 50);
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& c, const nav_msgs::msg::OccupancyGrid& map) {
  std::vector<CellIndex> neighbors;

  std::vector<std::pair<int,int>> directions = {
    {1, 0},   // right
    {-1, 0},  // left
    {0, 1},   // up
    {0, -1}   // down
  };
  for (auto d : directions) {
    CellIndex n(c.x + d.first, c.y + d.second);
    if (inBounds(n, map) && isFree(n, map)) {
      neighbors.push_back(n);
    }
  }
  return neighbors;
}

// Euclidian Distance
double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
  int dx = a.x - b.x;
  int dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy); // Euclidean distance
}

nav_msgs::msg::Path PlannerNode::reconstructPath(
    std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    CellIndex current,
    const nav_msgs::msg::OccupancyGrid& map) {
  
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  std::vector<geometry_msgs::msg::PoseStamped> poses;

  // Walk backwards until no parent
  while (came_from.find(current) != came_from.end()) {
    poses.push_back(mapToWorld(current, map));
    current = came_from[current];
  }
  poses.push_back(mapToWorld(current, map)); // add start

  // Reverse to get start → goal
  std::reverse(poses.begin(), poses.end());
  path.poses = poses;

  return path;
}

// Converts Grid Indices back into Real Coordinates
geometry_msgs::msg::PoseStamped PlannerNode::mapToWorld(const CellIndex& c, const nav_msgs::msg::OccupancyGrid& map) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = (c.x + 0.5) * map.info.resolution + map.info.origin.position.x;
  pose.pose.position.y = (c.y + 0.5) * map.info.resolution + map.info.origin.position.y;
  pose.pose.orientation.w = 1.0; // no rotation, just a position
  return pose;
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
