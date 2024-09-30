
# GPS Navigation Tools - Gridmap Creator

## Overview

The **GPS Navigation Tools** project provides a ROS 2 node for generating a dynamic gridmap based on GPS data. This node, `GridmapGpsCreator`, listens to GPS coordinates and constructs a gridmap where distances between points are visualized using a color gradient, indicating proximity from near to far.

## Key Features

- **GPS Integration**: Subscribes to GPS topics to receive positional data.
- **Dynamic Gridmap Generation**: Continuously updates a gridmap as new GPS data is received.
- **Color Gradient Visualization**: Maps the proximity of GPS points with a color scale from closest to farthest.
- **Customizable Parameters**: Users can configure grid resolution, map size, and color ranges.
- **ROS 2 Compatible**: Fully integrated within the ROS 2 ecosystem for seamless use in robotics applications.

## Node: `GridmapGpsCreator`

### Subscribed Topics

- `/gps_topic` (`sensor_msgs/msg/NavSatFix`): Receives GPS position data in latitude and longitude format.

### Published Topics

- `/grid_map` (`grid_map_msgs/msg/GridMap`): Publishes the dynamically generated gridmap that reflects the geographic area based on the received GPS data.

### Parameters

- `gps_topic_` (string): Topic to subscribe to for GPS data.
- `resolution_gridmap_` (double): Resolution of the gridmap in meters per cell. Default is `0.2`.
- `size_x_` (double): Size of the gridmap in the X direction (width) in meters. Default is `100.0`.
- `size_y_` (double): Size of the gridmap in the Y direction (height) in meters. Default is `100.0`.
- `color_unknown_` (float): Default color for unknown areas in the gridmap.
- `color_start_` (float): Color representing the closest GPS points.
- `color_end_` (float): Color representing the farthest GPS points.

## Requirements

Ubuntu 24.04 and ROS 2 Rolling distro are mandatory, Rolling can be installed, following this [steps](https://docs.ros.org/en/rolling/Installation.html)

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd ~/gps_nav_tools_ws/src
git clone https://github.com/aaggj/gps_nav_tools.git
```

Then build the package:

```bash
cd ~/gps_nav_tools_ws
colcon build
```

## Usage

Launch the node with default settings:

```bash
ros2 run gps_nav_tools gridmap_gps_creator
```

To customize the parameters, you can use a ROS 2 launch file or pass them directly via the command line.

```bash
ros2 run gps_nav_tools gridmap_gps_creator --ros-args -p gps_topic:=/custom_gps_topic -p resolution_gridmap:=0.1
```

## License

This project is licensed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).
