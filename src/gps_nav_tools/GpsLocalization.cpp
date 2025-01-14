#include "gps_nav_tools/GpsLocalization.hpp"

namespace gps_nav_tools
{
using std::placeholders::_1;
using namespace std::chrono_literals;


GpsLocalization::GpsLocalization(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("gps_localization", options)
{
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnTGpsLocalization::on_configure(const rclcpp_lifecycle::State & state)
{
  get_params();
  RCLCPP_INFO(get_logger(), "Parameters loaded");

  init_gridmap();
  RCLCPP_INFO(get_logger(), "Gridmap initialized");

  init_colors();
  RCLCPP_INFO(get_logger(), "Colors initialized");

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "gps_topic", 10, std::bind(&GpsLocalization::gps_callback, this, _1));

  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map_topic", 10, std::bind(&GpsLocalization::pc_callback, this, _1));

  gridmap_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);

  RCLCPP_INFO(get_logger(), "Configured");
  return nav2_util::CallbackReturn::SUCCESS;
}

CallbackReturnT
GpsLocalization::on_activate(const rclcpp_lifecycle::State & state)
{
  gridmap_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "Activating...");
  bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    RCLCPP_INFO(get_logger(), "use_sim_time = true");
  } else {
    RCLCPP_INFO(get_logger(), "use_sim_time = false");
  }

  reentrant_cg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  gps_timer_ = create_wall_timer(
    500ms, std::bind(&GpsLocalization::gps_receiver, this), reentrant_cg);
  map_timer_ = create_wall_timer(
    500ms, std::bind(&GpsLocalization::map_receiver, this), reentrant_cg);

  RCLCPP_INFO(get_logger(), "Activated");

  return nav2_util::CallbackReturn::SUCCESS;
}

CallbackReturnT
GpsLocalization::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  gps_timer_ = nullptr;
  map_timer_ = nullptr;

  reentrant_cg.reset();

  gridmap_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

CallbackReturnT
GpsLocalization::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  gps_sub_.reset();
  map_sub_.reset();
  gridmap_pub_.reset();

  gridmap_.reset();
  gps_timer_.reset();
  map_timer_.reset();

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

CallbackReturnT
GpsLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shut down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void GpsLocalization::get_params()
{
  this->declare_parameter<std::string>("gps_topic", "/gps/fix");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("map_frame_id", "map");
  this->declare_parameter<std::string>("gps_frame_id", "gps");
  this->declare_parameter<double>("resolution_gridmap", 0.2);
  this->declare_parameter<double>("size_x", 100.0);
  this->declare_parameter<double>("size_y", 100.0);
  this->declare_parameter<double>("origin_latitude", 40.2834611);
  this->declare_parameter<double>("origin_longitude", -3.8207427);
  this->declare_parameter<double>("origin_altitude", 733.4590000000001);
  this->declare_parameter<float>("color_near", 0.0);
  this->declare_parameter<float>("color_far", 1.0);
  this->declare_parameter<float>("color_unknown", -1.0);

  gps_topic_ = this->get_parameter("gps_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  gps_frame_id_ = this->get_parameter("gps_frame_id").as_string();
  resolution_gridmap_ = this->get_parameter("resolution_gridmap").as_double();
  size_x_ = this->get_parameter("size_x").as_double();
  size_y_ = this->get_parameter("size_y").as_double();
  origin_latitude_ = this->get_parameter("origin_latitude").as_double();
  origin_longitude_ = this->get_parameter("origin_longitude").as_double();
  origin_altitude_ = this->get_parameter("origin_altitude").as_double();
  color_near_ = this->get_parameter("color_near").as_float();
  color_far_ = this->get_parameter("color_far").as_float();
  color_unknown_ = this->get_parameter("color_unknown").as_float();
}

void GpsLocalization::init_gridmap()
{
  gridmap_ = std::make_shared<grid_map::GridMap>();
  gridmap_->setFrameId(map_frame_id_);
  gridmap_->setGeometry(
    grid_map::Length(size_x_, size_y_),
    resolution_gridmap_, grid_map::Position(0.0, 0.0));
  
  gridmap_->add("LatLong");
  gridmap_>add("Color");

  reset_gridmap();

  Eigen::Vector3i color_blue_v{0, 0, 255};
  float color_blue;
  grid_map::colorVectorToValue(color_blue_v, color_blue);

  for (grid_map::GridMapIterator it(*gridmap_); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    gridmap_->at("LatLong", index) = compress_lat_long(40.7128, -74.0060);
    gridmap_->at("Color", index) = color_blue;
  }

  
}

void GpsLocalization::reset_gridmap()
{
  gridmap_->clearAll();
  gridmap_->setBasicLayers({"LatLong", "Color"});
}

void GpsLocalization::update_gridmap(const double & latitude, const double & longitude, const int & x, const int & y)
{
  grid_map::Index index(x, y);
  if (gridmap_->isInside(index)) {
    gridmap_->at("LatLong", index) = compress_lat_long(latitude, longitude);
  }
}

void GpsLocalization::publish_gridmap(const builtin_interfaces::msg::Time & stamp)
{
  auto message = grid_map::GridMapRosConverter::toMessage(*gridmap_);
  message->header.stamp = stamp;
  gridmap_pub_->publish(std::move(message));
}

void GpsLocalization::gps_receiver(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
      !std::isnan(msg->latitude) && !std::isnan(msg->longitude)) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;

    // Convert GPS coordinates to grid map coordinates
    int x = static_cast<int>((latitude - origin_latitude_) / resolution_gridmap_);
    int y = static_cast<int>((longitude - origin_longitude_) / resolution_gridmap_);

    update_gridmap(latitude, longitude, x, y);
    publish_gridmap(this->now());
  }
}

void GpsLocalization::map_receiver(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, *gridmap_);
}

float GpsLocalization::calculate_color(const double & latitude, const double & longitude)
{
  double distance = std::sqrt(std::pow(latitude - origin_latitude_, 2) + std::pow(longitude - origin_longitude_, 2));
  double max_distance = std::sqrt(std::pow(size_x_, 2) + std::pow(size_y_, 2));
  return color_near_ + (color_far_ - color_near_) * (distance / max_distance);
}

uint64_t GpsLocalization::compress_lat_long(const double & latitude, const double & longitude)
{
  uint32_t lat = static_cast<uint32_t>((latitude + 90.0) * 1e6);
  uint32_t lon = static_cast<uint32_t>((longitude + 180.0) * 1e6);

  uint64_t lat_long = (static_cast<uint64_t>(lat) << 32) | static_cast<uint64_t>(lon);
  return lat_long;
}

std::tuple<double, double> GpsLocalization::decompress_lat_long(const uint64_t & lat_long)
{
  double latitude = (static_cast<double>(lat_long >> 32) - 90.0) / 1e6;
  double longitude = (static_cast<double>(lat_long & 0xFFFFFFFF) - 180.0) / 1e6;
  return std::make_tuple(latitude, longitude);
}

}  // namespace gps_nav_tools