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

#ifndef GPS_NAV_TOOLS_GPSLOCALIZATION_HPP_
#define GPS_NAV_TOOLS_GPSLOCALIZATION_HPP_

#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_ros/grid_map_ros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace gps_nav_tools
{
    class GpsLocalization : public nav2_util::LifecycleNode
    {
        public:
            explicit GpsLocalization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

            void init_gridmap();

            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
            nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

        protected:
            void get_params();
            void update_gridmap(const double & latitude, const double & longitude, const int & x, const int & y);
            void reset_gridmap();
            void publish_gridmap(const builtin_interfaces::msg::Time & stamp);
            void gps_receiver();
            void map_receiver();
            float calculate_color(const double & latitude, const double & longitude);
            uint64_t compress_lat_long(const double & latitude, const double & longitude);
            std::tuple<double,double> decompress_lat_long(const uint64_t & lat_long);
        private:
            rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
            rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
            std::shared_ptr<grid_map::GridMap> gridmap_;
            constexpr static double earth_radius_ {6371000.0};
            double resolution_gridmap_;
            double size_x_;
            double size_y_;
            double origin_latitude_;
            double origin_longitude_;
            double origin_altitude_;
            float color_near_;
            float color_far_;
            float color_unknown_;
            std::string map_frame_id_;
            std::string gps_frame_id_;
    };
}  // namespace gps_nav_tools

#endif  // GPS_NAV_TOOLS_GPSLOCALIZATION_HPP_
