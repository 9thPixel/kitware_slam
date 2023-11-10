//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2022-08-26
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

#include "AggregationNode.h"

// LidarSlam
#include <LidarSlam/Utilities.h>
#include <LidarSlam/PointCloudStorage.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>

// Boost
#include <boost/filesystem.hpp>

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
AggregationNode::AggregationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // ***************************************************************************
  // Init ROS publisher
  // aggregated points with specified density
  this->PointsPublisher = this->Nh.advertise<CloudS>("aggregated_cloud", 10, false);
  // Optional publisher
  this->ExtractSlice = this->PrivNh.param("slice/enable", false);
  if (this->ExtractSlice)
    this->SlicePublisher = this->Nh.advertise<CloudS>("slice_cloud", 10, false);

  // Init ROS subscribers
  // Lidar frame undistorted
  this->FrameSubscriber = this->Nh.subscribe("slam_registered_points", 1, &AggregationNode::Callback, this);
  // Optional lidar SLAM pose
  if (this->ExtractSlice)
    this->PoseSubscriber = this->Nh.subscribe("slam_odom", 1, &AggregationNode::PoseCallback, this);

  // Init service
  this->SaveService = nh.advertiseService("lidar_slam/save_pc", &AggregationNode::SavePointcloudService, this);

  // Init service
  this->RstService = nh.advertiseService("lidar_slam/reset", &AggregationNode::ResetService, this);

  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();

  // Slice parameters (needed to define the main voxel grid)
  this->TrajectoryMaxLength = this->PrivNh.param("slice/traj_length", 1.);
  // Compute the width of the slice to extract
  this->SliceWidth = this->PrivNh.param("slice/width", 0.2);
  // Set the max distance from the pose to compute a slice
  this->SliceMaxDist = this->PrivNh.param("slice/max_dist", 5.);

  // Get max size in meters
  float maxSize = this->PrivNh.param("max_size", 200.);
  // Set max size in voxels : the second dimension of
  // the rolling grid is used for the slice
  this->DenseMap->SetGridSize(maxSize / this->SliceMaxDist);

  // Set the voxels' sizes
  // Set the inner voxel size
  float leafSize = this->PrivNh.param("leaf_size", 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  // Set the outer voxel size,
  // it should be greater than the inner voxel
  this->DenseMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
  // Min number of frames seeing a voxel to extract it
  int minNbPointsPerVoxel = this->PrivNh.param("min_points_per_voxel", 2);
  this->DenseMap->SetMinFramesPerVoxel(minNbPointsPerVoxel);

  ROS_INFO_STREAM("Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const CloudS::Ptr registeredCloud)
{
  // Aggregated points from all frames
  this->DenseMap->Add(registeredCloud, false, false);
  this->Pointcloud = this->DenseMap->Get(true);
  this->Pointcloud->header = registeredCloud->header;
  // Publish them
  this->PointsPublisher.publish(this->Pointcloud);
}

//------------------------------------------------------------------------------
bool AggregationNode::SavePointcloudService(lidar_slam::save_pcRequest& req, lidar_slam::save_pcResponse& res)
{
  std::string outputPrefix = req.output_prefix_path.empty()? std::getenv("HOME") : req.output_prefix_path;
  boost::filesystem::path outputPrefixPath(outputPrefix);
  if (!boost::filesystem::exists(outputPrefixPath.parent_path()))
  {
    ROS_WARN_STREAM("Output folder does not exist, saving to home folder :" << std::getenv("HOME"));
    outputPrefixPath = boost::filesystem::path(std::getenv("HOME")) / boost::filesystem::path(outputPrefixPath.stem());
  }

  if (req.format > 2 || req.format < 0)
    req.format = 0;
  LidarSlam::PCDFormat f = static_cast<LidarSlam::PCDFormat>(req.format);
  std::cout << ros::Time().toSec() << std::endl;
  std::string outputFilePath = outputPrefixPath.string() + "_" + std::to_string(int(ros::Time::now().toSec())) + ".pcd";
  LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, *this->Pointcloud, f);
  ROS_INFO_STREAM("Pointcloud saved to " << outputFilePath);
  res.success = true;
  return true;
}

//------------------------------------------------------------------------------
bool AggregationNode::ResetService(lidar_slam::resetRequest& req, lidar_slam::resetResponse& res)
{
  this->DenseMap->Reset();
  ROS_INFO_STREAM("Resetting aggregation");
  res.success = true;
  return true;
}

//------------------------------------------------------------------------------
void AggregationNode::PoseCallback(const nav_msgs::Odometry& poseMsg)
{
  if (!this->ExtractSlice)
    return;

  // Store pose
  this->Positions.push_back(Utils::PoseMsgToIsometry(poseMsg.pose.pose).translation());
  Eigen::Vector3d& currPosition = this->Positions.back();
  while ((currPosition - this->Positions.front()).norm() > this->TrajectoryMaxLength)
    this->Positions.pop_front();

  if (this->Positions.size() < 2)
    return;

  // Deduce the motion direction
  Eigen::Vector3d motionDirection = (currPosition - this->Positions.front()).normalized();
  // Extract the outer voxel in which lays the new pose
  // To reduce the projection tests
  CloudS tmpCloud;
  tmpCloud.resize(1);
  tmpCloud[0].getVector3fMap() = currPosition.cast<float>();
  this->DenseMap->BuildSubMapKdTree(tmpCloud, this->MinSlicePtsWithoutMovObjects);
  CloudS::Ptr submap = this->DenseMap->GetSubMap();

  // Initialize the slice pointcloud
  CloudS slice;
  slice.header = this->Pointcloud->header;
  slice.reserve(submap->size());

  // Project them onto the slice plane
  for (const auto& pt : *submap)
  {
    // Check if point in sphere around the trajectory pose
    if ((pt.getVector3fMap().cast<double>() - currPosition).norm() > this->SliceMaxDist)
      continue;

    double ptDistance = (pt.getVector3fMap().cast<double>() - currPosition).dot(motionDirection);
    // Check if the point is not too far from the trajectory point
    if (std::abs(ptDistance) > this->SliceWidth)
      continue;

    slice.emplace_back(pt);
    slice.back().getVector3fMap() -= (ptDistance * motionDirection).cast<float>();
  }

  // Publish the slice points
  this->SlicePublisher.publish(slice);
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "aggregation");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  AggregationNode slam(nh, priv_nh);

  // Handle callbacks until shut down
  ros::spin();

  return 0;
}