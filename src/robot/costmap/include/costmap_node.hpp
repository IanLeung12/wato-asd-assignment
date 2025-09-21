#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // provides the core functionalities for ROS 2 nodes
#include "std_msgs/msg/string.hpp" // provides the standard String message type
#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Place callback function here
    void publishMessage();

    // Publisher

  private:
    // Declarations
    int occupied_ = 100;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    int inflation_radius_cells_;
    double inflation_radius_;

    std::vector<std::vector<int>> grid_costmap_;
    std::vector<std::pair<int,int>> obstacle_map_;

    void inflateCell(int obstacle_row, int obstacle_col);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
};

#endif 
