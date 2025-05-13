#include <vector>
#include <cmath>
#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
    RCLCPP_INFO(logger_, "Initializing costmap core");
    //Step 2: Initialize the Costmap
    resolution_ = 0.1;
    size_ = {100, 100};
    origin_ = {-5, -5};

    //Populate grid with zeroes according to dimensions
    grid_.assign(size_.second, std::vector<int>(size_.first, 0));
}

void CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
      // Before you compute x_grid, y_grid:
  if (scan->ranges.size() == 0) {
    RCLCPP_WARN(logger_, "Received empty scan!");
    return;
  }
    //Step 3: Convert LaserScan to Grid Coordinates
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range >= scan->range_min && range <= scan->range_max) {
            int x_grid, y_grid;
              // … compute x_grid, y_grid …
            convertToGrid(range, angle, x_grid, y_grid);
            //Step 4: Mark obstacles
            if ((x_grid >= 0 && x_grid < size_.first) && (y_grid >= 0 && y_grid < size_.second)) {
                grid_[y_grid][x_grid] = MAX_COST;
                obstacleLocations_.push_back({x_grid, y_grid});
            }
        }
    }
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    //Polar coordinate conversions
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x - origin_.first) / resolution_);
    y_grid = static_cast<int>((y - origin_.second) / resolution_);
}

//Step 5: Inflate obstacles
void CostmapCore::inflateObstacles() {
    int inflationRadius = static_cast<int>(1.0 / resolution_);
    //Iterate through all obstacles
    for (size_t i = 0; i < obstacleLocations_.size(); ++i) {
        std::pair<int, int> obstacle = obstacleLocations_[i];

        //Check all points within given radius
        for (int dx = inflationRadius * -1; dx < inflationRadius; ++dx) {
            //Out of bounds
            //Keep this here so we can just move on as soon as we see an invalid x val
            if ((obstacle.first + dx < 0) || (obstacle.first + dx >= size_.first)) {
                continue;
            }
            for (int dy = inflationRadius * -1; dy < inflationRadius; ++dy) {
                //Out of bounds
                if ((obstacle.second + dy < 0) || (obstacle.second + dy >= size_.second)) {
                    continue;
                }
                //On obstacle
                if (dx == 0 && dy == 0) {
                    continue;
                }
                else {
                    int nx = obstacle.first + dx;
                    int ny = obstacle.second + dy;
                    float distance = std::sqrt(dx * dx + dy * dy) * resolution_;

                    //Calculate cost and update if necessary
                    int cost = static_cast<int>(MAX_COST * (1 - (distance/inflationRadius)));
                    if (cost > grid_[ny][nx]) {
                        grid_[ny][nx] = cost;
                    }
                }
            }
        }
    }
}

//Step 6: Publish the costmap
nav_msgs::msg::OccupancyGrid CostmapCore::publishCostmap() {
    nav_msgs::msg::OccupancyGrid grid_msg;

    grid_msg.header.stamp = rclcpp::Clock().now();
    grid_msg.header.frame_id = "Costmap";

    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = size_.first;
    grid_msg.info.height = size_.second;
    grid_msg.info.origin.position.x = origin_.first;
    grid_msg.info.origin.position.y = origin_.second;
    grid_msg.info.origin.position.z = 0.0;

    // Convert std::vector<int> to std::vector<int8_t>
    std::vector<int8_t> flattenedGrid;
    flattenedGrid.reserve(size_.second * size_.first);
    for (int y = 0; y < size_.second; ++y) {
        for (int x = 0; x < size_.first; ++x) {
            flattenedGrid.push_back(grid_[y][x]);
        }
    }

    grid_msg.data = flattenedGrid;

    return grid_msg;
}

}