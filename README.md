# Cloud Merger Node

## Overview
The **Cloud Merger Node** is a ROS2 package for merging point clouds from two different LiDAR sensors, transforming them into a common frame, and publishing the merged point cloud. It uses message filters to synchronize the point clouds, transforms them to a target frame, and then merges them into a single cloud for further processing or visualization.

## Features
- Subscribes to two LiDAR point cloud topics.
- Transforms point clouds to a common target frame.
- Merges the point clouds.
- Publishes the merged point cloud as a ROS2 topic.
- Supports synchronized merging using message filters.

## Requirements
- ROS2 (Humble/Foxy)
- PCL (Point Cloud Library)
- `pcl_ros` for transforming and working with point clouds.
- `tf2_ros` for handling frame transformations.

## Installation

1. Clone this repository into your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone <repository_url>
    ```

2. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

## Parameters

The following parameters can be set in your launch file or via command line:

| Parameter          | Type   | Default         | Description                                    |
|--------------------|--------|-----------------|------------------------------------------------|
| `destination_frame`| string | `"base_link"`   | Target frame to which the point clouds are transformed. |
| `input_cloud_1`    | string | `"/front_lidar/points"` | Topic name for the first point cloud input.    |
| `input_cloud_2`    | string | `"/back_lidar/points"`  | Topic name for the second point cloud input.   |
| `merged_cloud`     | string | `"/merged_cloud"` | Topic name for the output merged point cloud. |

## Usage

1. **Running the node:**

    After building the package, you can run the node with:
    ```bash
    ros2 run cloud_merger_node cloud_merger_node
    ```

2. **Example launch file:**

    You can create a launch file to configure the parameters:

    ```xml
    <launch>
        <node
            name="cloud_merger_node"
            pkg="cloud_merger"
            exec="cloud_merger_node"
            output="screen">
            <param name="destination_frame" value="base_link" />
            <param name="input_cloud_1" value="/front_lidar/points" />
            <param name="input_cloud_2" value="/back_lidar/points" />
            <param name="merged_cloud" value="/merged_cloud" />
        </node>
    </launch>
    ```

3. **Viewing the merged point cloud:**

    You can use `rviz2` to visualize the merged point cloud:
    ```bash
    rviz2
    ```

    - Add a new display for "PointCloud2".
    - Set the topic to `/merged_cloud` to see the merged result.

## Debugging

The node provides useful debug information. To enable `DEBUG` level logging, you can use the following command:

```bash
ros2 run cloud_merger_node cloud_merger_node --ros-args --log-level debug
