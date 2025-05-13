#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void MapMemoryCore::initMapMemory(
  double resolution, 
  int width, 
  int height, 
  geometry_msgs::msg::Pose origin
) {
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;
  global_map_->info.height = height;
  global_map_->info.origin = origin;
  global_map_->data.assign(width * height, 0);

  RCLCPP_INFO(logger_, "Global Map initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void MapMemoryCore::updateMap(
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
  double robot_x, double robot_y, double robot_angle
) {
  // Get local costmap specs
  double local_res = local_costmap->info.resolution;
  double local_origin_x = local_costmap->info.origin.position.x;
  double local_origin_y = local_costmap->info.origin.position.y;
  unsigned int local_w = local_costmap->info.width;
  unsigned int local_h = local_costmap->info.height;
  const auto & local_data = local_costmap->data;

  //Get global costmap specs
  double origin_x = global_map_->info.origin.position.x;
  double origin_y = global_map_->info.origin.position.y;
  double resolution = global_map_->info.resolution;

  // For each cell in local costmap, transform to sim_world
  for (unsigned int j = 0; j < local_h; ++j)
  {
    for (unsigned int i = 0; i < local_w; ++i)
    {
      int8_t cost_val = local_data[j * local_w + i];
      //Invalid
      if (cost_val < 0) {
        continue;
      }

      //Compute the center of the local cell (i,j) in robot-frame meters
      double lx = local_origin_x + (i + 0.5) * local_res;
      double ly = local_origin_y + (j + 0.5) * local_res;

      //Polar coordinate conversion
      double cos_t = std::cos(robot_angle);
      double sin_t = std::sin(robot_angle);
      double wx = robot_x + (lx * cos_t - ly * sin_t);
      double wy = robot_y + (lx * sin_t + ly * cos_t);

      //World point lies before map origin (out of bounds)
      if (wx < origin_x || wy < origin_y) {
        continue;
      }

      //Convert world points to grid indices
      int gx, gy;
      gx = static_cast<int>((wx - origin_x) / resolution);
      gy = static_cast<int>((wy - origin_y) / resolution);

      //Grid indices out of bounds
      if ((gx < 0 || gx >= static_cast<int>(global_map_->info.width))
          || (gy < 0 || gy >= static_cast<int>(global_map_->info.height))) {
        continue;
    }

      //Reference corresponding global map cell to update in place
      int8_t &global_val = global_map_->data[gy * global_map_->info.width + gx];
      
      int current_global_cost = (global_val < 0) ? 0 : global_val; 

      //Keep most costly value, update global grid cell
      int merged_cost = std::max(current_global_cost, static_cast<int>(cost_val));
      global_val = static_cast<int8_t>(merged_cost);
    }
  }
}

// Retrieves map data
nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMapData() const {
  return global_map_;
}

} 