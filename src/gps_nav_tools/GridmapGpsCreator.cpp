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

#include "gps_nav_tools/GridmapGpsCreator.hpp"

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "rclcpp/rclcpp.hpp"

#include <algorithm>


namespace gps_nav_tools
{

using std::placeholders::_1;
using namespace std::chrono_literals;

GridmapGpsCreator::GridmapGpsCreator(const rclcpp::NodeOptions & options)
: Node("gridmap_gps_creator_node", options)
{
  get_params();
  init_gridmap();
  init_colors();

  auto gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic_, 10, std::bind(&GridmapGpsCreator::gps_callback, this, _1));
  gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);

}

void 
GridmapGpsCreator::get_params()
{
  this->declare_parameter<std::string>("gps_topic", "/gps/fix");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<double>("resolution", 0.2);
  this->declare_parameter<double>("size_x", 100.0);
  this->declare_parameter<double>("size_y", 100.0);
  this->declare_parameter<double>("origin_latitude", 40.2834611);
  this->declare_parameter<double>("origin_longitude", -3.8207427);
  this->declare_parameter<double>("origin_altitude", 733.4590000000001);

  gps_topic_ = this->get_parameter("gps_topic").as_string();
  map_frame_id_ = this->get_parameter("map_frame").as_string();
  resolution_gridmap_ = this->get_parameter("resolution").as_double();
  size_x_ = this->get_parameter("size_x").as_double();
  size_y_ = this->get_parameter("size_y").as_double();
  origin_latitude_ = this->get_parameter("origin_latitude").as_double();
  origin_longitude_ = this->get_parameter("origin_longitude").as_double();
  origin_altitude_ = this->get_parameter("origin_altitude").as_double();
}

void GridmapGpsCreator::init_gridmap()
{
  // Grid map size and frame initialization
  gridmap_ = std::make_shared<grid_map::GridMap>();
  gridmap_->setFrameId(map_frame_id_);
  gridmap_->setGeometry(
    grid_map::Length(size_x_, size_y_),
    resolution_gridmap_, grid_map::Position(0.0, 0.0));
  
  //Layers initialization
  gridmap_->add("Latitude");
  gridmap_->add("Longitude");
  gridmap_->add("Altitude");
  gridmap_->add("Color");

  reset_gridmap();
}

void GridmapGpsCreator::init_colors()
{
  color_unknown_ = 0.0;  // color for unknown areas
  color_start_ = 0.5;    // start color value for closer points
  color_end_ = 1.0;      // end color value farther points
}

void GridmapGpsCreator::reset_gridmap()
{
  gridmap_->clearAll();
  gridmap_->setBasicLayers({"LatLong", "Elevation", "Color"});
}

void GridmapGpsCreator::update_gridmap(const double & latitude,
  const double & longitude, const double & altitude)
{
  // Transform lat/lon to gridmap x/y coordinates and update gridmap
  auto [x, y] = gpsToGrid(latitude, longitude);
  grid_map::Index index;
  if (gridmap_->getIndex(grid_map::Position(x, y), index)) {
    gridmap_->at("Latitude", index) = latitude;
    gridmap_->at("Longitude", index) = longitude;
    gridmap_->at("Altitude", index) = altitude;
    gridmap_->at("color", index) = calculate_color(latitude, longitude);
  }
}

void
GridmapGpsCreator::publish_gridmap(const builtin_interfaces::msg::Time & stamp)
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg;
  msg = grid_map::GridMapRosConverter::toMessage(*gridmap_);
  msg->header.frame_id = map_frame_id_;
  msg->header.stamp = stamp;

  gridmap_pub_->publish(std::move(msg));  
  
}

void
GridmapGpsCreator::gps_callback(const sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  if(!gps_ok(msg)) {
    RCLCPP_WARN(get_logger(), "Invalid GPS data");
    return;
  }
  GpsData gpsData{msg->latitude, msg->longitude, msg->altitude};

  update_gridmap(gpsData.latitude, gpsData.longitude, gpsData.altitude);
  publish_gridmap(this->now());
}


float 
GridmapGpsCreator::calculate_color(const double & latitude, const double & longitude)
{
  double delta_lat = latitude - origin_latitude_;
  double delta_lon = longitude - origin_longitude_;

  double distance_squared = std::pow(delta_lat, 2) + std::pow(delta_lon, 2);

  double max_distance_squared = std::pow(size_x_, 2) + std::pow(size_y_, 2);
  double normalized_distance = distance_squared / max_distance_squared;

  return color_start_ + (color_end_ - color_start_) * normalized_distance;
}

}  // namespace gps_nav_tools
