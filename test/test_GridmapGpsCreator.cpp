#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "gps_nav_tools/GridmapGpsCreator.hpp"

class GridmapGpsCreatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<gps_nav_tools::GridmapGpsCreator>(rclcpp::NodeOptions());
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<gps_nav_tools::GridmapGpsCreator> node_;
};

TEST_F(GridmapGpsCreatorTest, TestGetParams)
{
    node_->get_params();
    EXPECT_EQ(node_->gps_topic_, "/gps/fix");
    EXPECT_EQ(node_->map_frame_id_, "map");
    EXPECT_DOUBLE_EQ(node_->resolution_gridmap_, 0.2);
    EXPECT_DOUBLE_EQ(node_->size_x_, 100.0);
    EXPECT_DOUBLE_EQ(node_->size_y_, 100.0);
    EXPECT_DOUBLE_EQ(node_->origin_latitude_, 40.2834611);
    EXPECT_DOUBLE_EQ(node_->origin_longitude_, -3.8207427);
    EXPECT_DOUBLE_EQ(node_->origin_altitude_, 733.4590000000001);
}

TEST_F(GridmapGpsCreatorTest, TestInitGridmap)
{
    node_->init_gridmap();
    EXPECT_EQ(node_->gridmap_->getFrameId(), "map");
    EXPECT_EQ(node_->gridmap_->getLayers().size(), 4);
    EXPECT_TRUE(node_->gridmap_->exists("Latitude"));
    EXPECT_TRUE(node_->gridmap_->exists("Longitude"));
    EXPECT_TRUE(node_->gridmap_->exists("Altitude"));
    EXPECT_TRUE(node_->gridmap_->exists("Color"));
}

TEST_F(GridmapGpsCreatorTest, TestInitColors)
{
    node_->init_colors();
    EXPECT_DOUBLE_EQ(node_->color_unknown_, 0.0);
    EXPECT_DOUBLE_EQ(node_->color_start_, 0.5);
    EXPECT_DOUBLE_EQ(node_->color_end_, 1.0);
}

TEST_F(GridmapGpsCreatorTest, TestUpdateGridmap)
{
    node_->init_gridmap();
    double latitude = 40.2834611;
    double longitude = -3.8207427;
    double altitude = 733.4590000000001;
    node_->update_gridmap(latitude, longitude, altitude);

    auto [x, y] = node_->gpsToGrid(latitude, longitude);
    grid_map::Index index;
    if (node_->gridmap_->getIndex(grid_map::Position(x, y), index)) {
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Latitude", index), latitude);
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Longitude", index), longitude);
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Altitude", index), altitude);
    }
}

TEST_F(GridmapGpsCreatorTest, TestGpsCallback)
{
    auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    msg->latitude = 40.2834611;
    msg->longitude = -3.8207427;
    msg->altitude = 733.4590000000001;

    node_->gps_callback(std::move(msg));

    auto [x, y] = node_->gpsToGrid(40.2834611, -3.8207427);
    grid_map::Index index;
    if (node_->gridmap_->getIndex(grid_map::Position(x, y), index)) {
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Latitude", index), 40.2834611);
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Longitude", index), -3.8207427);
        EXPECT_DOUBLE_EQ(node_->gridmap_->at("Altitude", index), 733.4590000000001);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}