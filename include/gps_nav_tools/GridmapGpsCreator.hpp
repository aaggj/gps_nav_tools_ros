// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GPS_NAV_TOOLS_GRIDMAPCREATORNODE_HPP_
#define GPS_NAV_TOOLS_GRIDMAPCREATORNODE_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace gps_nav_tools
{

class GridmapGpsCreator : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GridmapGpsCreator);

  explicit GridmapGpsCreator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  void get_params();
  void init_gridmap();
  void init_colors();
  void reset_gridmap();
  void update_gridmap(
    const double & x,
    const double & y,
    const latitudelongitude & latlon,
  )
  void publish_gridmap();

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;

  void gps_callback(const sensor_msgs::msg::NavSatFix::UniquePtr msg);

  std::string gps_topic_;

  double resolution_gridmap_ {0.2};
  double size_x_ {100.0};
  double size_y_ {100.0};

  std::shared_ptr<grid_map::GridMap> gridmap_;

  float color_unknown_;
  float color_start_;
  float color_end_;
};

} // namespace gps_nav_tools

#endif  // GPS_NAV_TOOLS_GRIDMAPCREATORNODE_HPP_