# lidar_conversions

## Description

Helper package to convert raw pointclouds output by common LiDAR drivers to the pointcloud format expected by the SLAM algorithm.

The SLAM algorithm expects input pointclouds as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise aquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **device_id** (`uint8`), **label** (`uint8`) : optional inputs, not yet used.

Especially, the point-wise timestamps are necessary if undistortion is enabled in SLAM. The nodes of this package are able to compute approximate timestamps when those are not available in the input pointcloud.

Currently, this package implements the following nodes :
- **velodyne_conversion_node** : converts pointclouds output by Velodyne spinning sensors using the [ROS Velodyne driver](https://github.com/ros-drivers/velodyne) to SLAM pointcloud format.
- **robosense_conversion_node** : converts pointclouds output by RoboSense spinning sensors using the [ROS RoboSense-LiDAR driver](https://github.com/RoboSense-LiDAR/ros_rslidar) to SLAM pointcloud format. This has been tested only with RS16 sensor, and could need additional changes to support other RS sensors.

## Usage

Direct usage :

```bash
rosrun lidar_conversions velodyne_conversion_node
```

Example of minimal launchfile for a multi-lidar setup:

```xml
<launch>
  <!-- LiDAR pointclouds conversions.
       The parameters are only used to generate approximate point-wise timestamps if 'time' field is not usable.
       These parameters should be set to the same values as ROS Velodyne/RSLidar drivers'. -->

  <node name="velodyne_conversion" pkg="lidar_conversions" type="velodyne_conversion_node" output="screen">
    <param name="rpm" value="600."/>
    <param name="timestamp_first_packet" value="false"/>
    <remap from="lidar_points" to="velodyne_lidar_points"/>
  </node>

  <node name="robosense_conversion" pkg="lidar_conversions" type="robosense_conversion_node" output="screen">
    <param name="rpm" value="600."/>
    <remap from="lidar_points" to="robosense_lidar_points"/>
  </node>
</launch>
```