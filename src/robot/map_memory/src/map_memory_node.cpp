#include "map_memory_node.hpp"

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(5.0) {
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
 
        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));

        // setup 30x30 global map
        global_map_.header.frame_id = "sim_world";
        global_map_.info.width = 300;
        global_map_.info.height = 300;
        global_map_.info.resolution = 0.1;
        global_map_.info.origin.position.x = global_map_.info.width * global_map_.info.resolution / -2;
        global_map_.info.origin.position.y = global_map_.info.height * global_map_.info.resolution / -2;
        global_map_.info.origin.orientation.w = 1.0;
        global_map_.data.resize(300 * 300, 0);
    }
 
private:
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y, last_yaw;
    const double distance_threshold = 1.5;
    bool costmap_updated_ = false;
 
    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Store the latest costmap
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }
 
    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
 
        // quaternion to yaw
        geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
        last_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // Compute distance traveled
        double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
        if (distance >= distance_threshold) {
            last_x = x;
            last_y = y;
            should_update_map_ = true;
        }
    }
 
    // Timer-based map update
    void updateMap() {
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
        }
    }
 
     // Integrate the latest costmap into the global map
    void integrateCostmap() {
        // Transform and merge the latest costmap into the global map
        // (Implementation would handle grid alignment and merging logic)

        for (int cm_y = 0; cm_y < latest_costmap_.info.height; ++cm_y) {
            for (int cm_x = 0; cm_x < latest_costmap_.info.width; ++cm_x) {
                int cost = latest_costmap_.data[cm_y * latest_costmap_.info.width + cm_x];

                if (cost < 0) {
                    continue;
                }

                // convert costmap indices to local coordinates
                double local_x = latest_costmap_.info.origin.position.x + (cm_x + 0.5) * latest_costmap_.info.resolution;
                double local_y = latest_costmap_.info.origin.position.y + (cm_y + 0.5) * latest_costmap_.info.resolution;

                // convert local coordinates to global coordinates
                double world_x = std::cos(last_yaw) * local_x - std::sin(last_yaw) * local_y + last_x;
                double world_y = std::sin(last_yaw) * local_x + std::cos(last_yaw) * local_y + last_y;

                // convert global coordinates to global indices
                int global_x = static_cast<int>(std::round((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution));
                int global_y = static_cast<int>(std::round((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution));

                // make sure within bounds
                if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) &&
                    global_y >= 0 && global_y < static_cast<int>(global_map_.info.height)) {
                    int global_index = global_y * global_map_.info.width + global_x;

                    if (cost > global_map_.data[global_index]) {
                        global_map_.data[global_index] = cost;
                    }
                }
            }
        }
    }
 
    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = true; // update once on initialization
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
