#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <utility>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "rclcpp/rclcpp.hpp"

namespace robot
{
  #define MAX_COST 100

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid publishCostmap();

  private:
    rclcpp::Logger logger_;
    float resolution_;
    std::pair<int, int> origin_;
    std::pair<int, int> size_;
    std::vector<std::vector<int>> grid_;
    std::vector<std::pair<int, int>> obstacleLocations_;
};

}  

#endif  