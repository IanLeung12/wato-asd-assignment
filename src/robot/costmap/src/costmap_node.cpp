#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10);

  // Initialize costmap parameters
  resolution_ = 0.1;
  width_ = 300;
  height_ = 300;
  origin_x_ = -1*((width_ * resolution_)/2.0);
  origin_y_ = -1*((height_ * resolution_)/2.0);
  inflation_radius_ = 1.0; 
  inflation_radius_cells_ = static_cast<int>(inflation_radius_ / resolution_);
  grid_costmap_.resize(height_, std::vector<int>(width_, 0)); 

}

// Callback function to process LIDAR data and update costmap
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Reset costmap and obstacle map
  for (int i = 0; i < height_; i++) {
  std::fill(grid_costmap_[i].begin(), grid_costmap_[i].end(), 0);
  }
  obstacle_map_.clear();

  // Process LIDAR data
  for (int i = 0; i <msg->ranges.size(); i++){
    double x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);
    double y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);

    int column = (x - origin_x_) / resolution_;
    int row = (y - origin_y_) / resolution_;

    // Marking as occupied in costmap
    if ((row >= 0 && row < height_) && (column >=0 && column < width_)){
      grid_costmap_[row][column] = 100; // Occupied
      obstacle_map_.push_back({row, column});
    }
  }
  // Inflate obstacles
  for (auto [row, col] : obstacle_map_) {
    // Calls the inflate function
    inflateCell(row, col);
  }
  // Prepare occupancy grid message
  nav_msgs::msg::OccupancyGrid costmap_msg;

  //Publishing the costmap
  costmap_msg.header.stamp = this->now();
  costmap_msg.header.frame_id = "map";
  costmap_msg.info.resolution = resolution_;
  costmap_msg.info.width = width_;
  costmap_msg.info.height = height_;
  costmap_msg.info.origin.position.x = origin_x_;
  costmap_msg.info.origin.position.y = origin_y_;
  costmap_msg.info.origin.orientation.w = 1.0;

  // Fill in the data
  costmap_msg.data.resize(width_ * height_);
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      costmap_msg.data[i * width_ +  j] = grid_costmap_[i][j];
    }
  }
  // Publish the costmap
  costmap_publisher_->publish(costmap_msg);
}

// Inflate the cost around an obstacle cell
void CostmapNode::inflateCell(int obstacle_row, int obstacle_column){
  for (int row_offset = -inflation_radius_cells_; row_offset <= inflation_radius_cells_; row_offset++){
    for (int column_offset = -inflation_radius_cells_; column_offset <= inflation_radius_cells_; column_offset++){
      // Check bounds
      int neighbour_row = obstacle_row + row_offset;
      int neighbour_column = obstacle_column + column_offset;

      // Skip if out of bounds
      if (neighbour_row < 0 || neighbour_row >= height_ || neighbour_column < 0 || neighbour_column >= width_){
        continue; 
      }
      // Converting cell location to distance in meters
      double dx = column_offset * resolution_;
      double dy = row_offset * resolution_;
      double distance = sqrt((dx * dx)+ (dy * dy));

      // If within inflation radius, update cost
      if (distance <= inflation_radius_){
        int new_cost = occupied_ * (1 - (distance / inflation_radius_));
        if (new_cost > grid_costmap_[neighbour_row][neighbour_column]){
          grid_costmap_[neighbour_row][neighbour_column] = new_cost;
        }
      }
    }
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

