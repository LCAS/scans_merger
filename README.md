# Cloud Merger
[![Humble Build Main](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-humble.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-humble.yml) [![Jazzy Build Main](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-jazzy.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-jazzy.yml) [![Kilted Build Main](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-kilted.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-kilted.yml) [![Rolling Build Main](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-rolling.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_scan_merger/actions/workflows/build-rolling.yml)


The **Cloud Merger** is a ROS2 package for merging point clouds from two different LiDAR sensors, transforming them into a common frame, and publishing the merged point cloud. It uses message filters to synchronize the point clouds, transforms them to a target frame, and then merges them into a single cloud.

# License
The contents are licensed under the [Apache 2.0 license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking/cloning this repository for your application. Please open an issue in order to get in touch with us.

# Usage
## Parameters

The following parameters can be set in your launch file or via command line:

| Parameter          | Type   | Default         | Description                                    |
|--------------------|--------|-----------------|------------------------------------------------|
| `destination_frame`| string | `"base_link"`   | Target frame to which the point clouds are transformed. |
| `input_cloud_1`    | string | `"/front_lidar/points"` | Topic name for the first point cloud input.    |
| `input_cloud_2`    | string | `"/back_lidar/points"`  | Topic name for the second point cloud input.   |
| `merged_cloud`     | string | `"/merged_cloud"` | Topic name for the output merged point cloud. |

## Debugging

The node provides useful debug information. To enable `DEBUG` level logging, you can use the following command:

```bash
ros2 run cloud_merger_node cloud_merger_node --ros-args --log-level debug
```

# Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)
