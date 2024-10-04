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

#ifndef GPS_NAV_TOOLS_GRIDMAPCREATOR_HPP_
#define GPS_NAV_TOOLS_GRIDMAPCREATOR_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_ros/grid_map_ros.hpp"


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace gps_nav_tools
{

class GridmapGpsCreator : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GridmapGpsCreator)

  struct GpsData
  {
    double latitude;
    double longitude;
    double altitude;
  };

  explicit GridmapGpsCreator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  void get_params();
  void init_gridmap();
  void init_colors();
  void reset_gridmap();
  void update_gridmap(
    const double & latitude, const double & longitude, const double & altitude);
  void publish_gridmap(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;

  void gps_callback(const sensor_msgs::msg::NavSatFix::UniquePtr msg);

  inline bool gps_ok(const sensor_msgs::msg::NavSatFix::UniquePtr & msg)
  {
    return (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
      !std::isnan(msg->altitude) && !std::isnan(msg->latitude) &&
      !std::isnan(msg->longitude));
  }
  float calculate_color(const double & latitude, const double & longitude);

  inline std::pair<int, int> gpsToGrid(double latituted, double longitude)
  {
    return std::make_pair(
      static_cast<int>((latituted - origin_latitude_) / resolution_gridmap_),
      static_cast<int>((longitude - origin_longitude_) / resolution_gridmap_));
  }

  std::string gps_topic_;
  std::string map_frame_id_;

  double resolution_gridmap_ {0.2};
  double size_x_ {100.0};
  double size_y_ {100.0};
  double origin_latitude_, origin_longitude_, origin_altitude_;

  std::shared_ptr<grid_map::GridMap> gridmap_;

  float color_unknown_, color_start_, color_end_;
};

} // namespace gps_nav_tools

#endif  // GPS_NAV_TOOLS_GRIDMAPCREATOR_HPP_