//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2019-08-28
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
//==============================================================================

#ifndef AGGREGATION_NODE_H
#define AGGREGATION_NODE_H

// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// LidarSlam
#include <LidarSlam/LidarPoint.h>
#include <LidarSlam/RollingGrid.h>

#include "lidar_slam/save_pc.h"
#include "lidar_slam/reset.h"

class AggregationNode
{
public:

  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  AggregationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     New main frame callback, aggregating frames
   * @param[in] cloud New registered frame published by the LidarSlamNode
   *
   * Input pointcloud must have following fields :
   *  - x, y, z (float): point coordinates
   *  - time (double): time offset to add to the pointcloud header timestamp to
   *    get approximate point-wise acquisition timestamp
   *  - intensity (float): intensity/reflectivity of the point
   *  - laser_id (uint16): numeric identifier of the laser ring that shot this point.
   *    The lowest/bottom laser ring should be 0, and it should increase upward.
   *  - device_id (uint8): numeric identifier of the LiDAR device/sensor.
   *    This id should be the same for all points of the cloud acquired by the same sensor.
   *  - label (uint8): optional input, not yet used.
   */
  void Callback(const CloudS::Ptr registeredCloud);

  bool SavePointcloudService(lidar_slam::save_pcRequest& req, lidar_slam::save_pcResponse& res);

  bool ResetService(lidar_slam::resetRequest& req, lidar_slam::resetResponse& res);

private:

  // ROS node handles, subscribers and publishers
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber FrameSubscriber;
  ros::Publisher PointsPublisher;
  ros::ServiceServer SaveService;
  ros::ServiceServer RstService;

  // Dense map containing aggregated points from all frames
  std::shared_ptr<LidarSlam::RollingGrid> DenseMap;
  CloudS::Ptr Pointcloud;
};

#endif // AGGREGATION_NODE_H