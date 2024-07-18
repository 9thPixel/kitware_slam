//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2022-09-02
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

// Boost
#include <boost/filesystem.hpp>

// PCL
#include <pcl/common/transforms.h>

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
AggregationNode::AggregationNode(std::string name_node, const rclcpp::NodeOptions& options)
  : rclcpp::Node(name_node, options)
{
  // ***************************************************************************
  // Init ROS publisher
  // aggregated points with specified density
  this->PointsPublisher = this->create_publisher<Pcl2_msg>("aggregated_cloud", 10);
  // Optional publisher
  this->get_parameter_or<bool>("slice.enable", this->DoExtractSlice, false);
  if (this->DoExtractSlice)
  {
    this->SlicePublisher = this->create_publisher<Pcl2_msg>("slice_cloud", 10);
    this->SliceAreaPublisher = this->create_publisher<std_msgs::msg::Float64>("slice_area", 10);
  }

  // Init ROS subscribers
  // Lidar frame undistorted
  this->FrameSubscriber = this->create_subscription<Pcl2_msg>(
            "slam_registered_points", 1, std::bind(&AggregationNode::Callback, this, std::placeholders::_1));
  // Optional lidar SLAM pose
  this->PoseSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "slam_odom", 1, std::bind(&AggregationNode::PoseCallback, this, std::placeholders::_1));

  // Init service
  this->SaveService = this->create_service<lidar_slam::srv::SavePc>(
      "lidar_slam/save_pc",
      std::bind(&AggregationNode::SavePointcloudService, this, std::placeholders::_1, std::placeholders::_2));

  // Init service
  this->RstService = this->create_service<lidar_slam::srv::Reset>(
      "lidar_slam/reset",
      std::bind(&AggregationNode::ResetService, this, std::placeholders::_1, std::placeholders::_2));


  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();
  this->RefMap = std::make_shared<LidarSlam::RollingGrid>();

  // Slice parameters (needed to define the main voxel grid)
  this->get_parameter_or<double>("slice.traj_length", this->TrajectoryMaxLength, 1.);
  // Compute the width of the slice to extract
  this->get_parameter_or<double>("slice.width", this->SliceWidth, 0.2);
  // Set the max distance from the pose to compute a slice
  this->get_parameter_or<double>("slice.max_dist", this->SliceMaxDist, 5.);
  double angleStepDeg;
  this->get_parameter_or<double>("slice.angle_resolution", angleStepDeg, 3.);
  this->AngleStep = (M_PI / 180.) * angleStepDeg;

  // Horizontal slice extraction parameters
  this->get_parameter_or<bool>("z_slice.enable", this->DoExtractZslice, false);
  this->get_parameter_or<double>("z_slice.width", this->ZsliceWidth, 0.1);
  this->get_parameter_or<double>("z_slice.height_position", this->ZsliceHeightPosition, -0.5);
  this->get_parameter_or<bool>("z_slice.invert", this->InvertZsliceExtraction, false);

  // Obstacle extraction parameters
  this->get_parameter_or<bool>("obstacle.enable", this->DoExtractObstacle, false);
  std::string refMapPath;
  this->get_parameter<std::string>("obstacle.ref_map_path", refMapPath);
  this->get_parameter_or<bool>("obstacle.enable", this->DoExtractObstacle, false);
  double decayTime;
  this->get_parameter_or<double>("obstacle.decay_time", decayTime, 1000.);
  this->DenseMap->SetDecayingThreshold(decayTime);
  bool publishGrid;
  this->get_parameter_or<bool>("obstacle.publish_occupancy_grid", publishGrid, false);

  if (this->DoExtractObstacle)
  {
    if (publishGrid)
      this->OccupancyPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacles/occupancy_grid", 10);
  }

  // Get max size in meters
  double maxSize;
  this->get_parameter_or<double>("max_size", maxSize, 200.);
  // Set max size in voxels : the second dimension of
  // the rolling grid is used for the slice
  this->DenseMap->SetGridSize(maxSize / this->SliceMaxDist);
  this->RefMap->SetGridSize(maxSize / this->SliceMaxDist);

  // Set the voxels' sizes
  // Set the inner voxel size
  float leafSize;
  this->get_parameter_or<float>("leaf_size", leafSize, 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  this->RefMap->SetLeafSize(leafSize);
  // Set the outer voxel size,
  // it should be greater than the inner voxel
  this->DenseMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
  this->RefMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
  // Min number of frames seeing a voxel to extract it
  int minNbPointsPerVoxel;
  this->get_parameter_or<int>("min_points_per_voxel", minNbPointsPerVoxel, 2);
  this->DenseMap->SetMinFramesPerVoxel(minNbPointsPerVoxel);
  // Set minimal distance of the points to the trajectory
  // Allows to remove traces from the map
  this->get_parameter_or<double>("min_dist_around_trajectory", this->MinDistAroundTrajectory, 1.);
  // Set maximal distance for input points
  this->get_parameter_or<double>("max_dist_around_trajectory", this->MaxDistAroundTrajectory, -1.);

  // Load ref map
  if (!refMapPath.empty())
  {
    this->LoadRefMap(refMapPath);
    this->Pointcloud = this->RefMap->Get();
    this->Pointcloud->header.frame_id = "odom";
    Pcl2_msg aggregatedCloudMsg;
    pcl::toROSMsg(*this->Pointcloud, aggregatedCloudMsg);
    this->PointsPublisher->publish(aggregatedCloudMsg);
  }

  if (this->DoExtractObstacle)
  {
    if (publishGrid)
      this->OccupancyPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacles/occupancy_grid", 10);
    this->MarkerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_boxes", 10);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const Pcl2_msg& registeredCloudMsg)
{
  //Convert msg to PointCloud
  CloudS::Ptr registeredCloud = std::make_shared<CloudS>();
  pcl::fromROSMsg(registeredCloudMsg, *registeredCloud);
  double currentTime = LidarSlam::Utils::PclStampToSec(registeredCloud->header.stamp);

  // Clear old points
  this->DenseMap->ClearPoints(currentTime);

  // Remove the farthest points if required
  CloudS::Ptr closest;
  if (this->MaxDistAroundTrajectory > 0)
  {
    closest.reset(new CloudS);
    closest->reserve(registeredCloud->size());
    closest->header = registeredCloud->header;
    for (const auto& pt : *registeredCloud)
    {
      if ((pt.getVector3fMap() - this->CurrentPose.translation().cast<float>()).norm() < this->MaxDistAroundTrajectory)
        closest->emplace_back(pt);
    }
  }
  else
    closest = registeredCloud;

  // Extract/reject a slice in z axis if required
  CloudS::Ptr preprocessedCloud;
  if (this->DoExtractZslice)
  {
    preprocessedCloud.reset(new CloudS);
    preprocessedCloud->reserve(closest->size());
    preprocessedCloud->header = closest->header;
    // Extract the z slice points
    this->ExtractZslice(*closest, *preprocessedCloud);
  }
  else
    preprocessedCloud = closest;

  // Extract obstacle and clusters of obstacles
  if (this->DoExtractObstacle)
  {
    // Label points w.r.t reference map
    // Old points are labeled as FIXED
    // Others are labeled as DEFAULT
    this->RefMap->LabelNewPoints(preprocessedCloud, true);
    // Extract new points
    CloudS::Ptr newPoints = std::make_shared<CloudS>();
    newPoints->reserve(preprocessedCloud->size());
    newPoints->header = preprocessedCloud->header;
    for (auto& pt : *preprocessedCloud)
    {
      if (pt.label != static_cast<uint16_t>(LidarSlam::LidarPointLabel::FIXED))
        newPoints->emplace_back(pt);
    }

    // Label obstacles with cluster indices
    this->LabelObstacle(*newPoints, currentTime);
    preprocessedCloud = newPoints;

    // publish occupancyGrid
    if (this->OccupancyPublisher)
    {
      auto oGMsg = nav_msgs::msg::OccupancyGrid();
      oGMsg.header.stamp = rclcpp::Clock().now();
      oGMsg.header.frame_id = registeredCloudMsg.header.frame_id;
      oGMsg.info.map_load_time = rclcpp::Clock().now();
      oGMsg.info.resolution = this->DenseMap->GetLeafSize();
      oGMsg.info.width = this->ObstaclesGrid.Width;
      oGMsg.info.height = this->ObstaclesGrid.Height;
      oGMsg.info.origin.position.x = this->ObstaclesGrid.Origin.x();
      oGMsg.info.origin.position.y = this->ObstaclesGrid.Origin.y();
      oGMsg.info.origin.position.z = this->ZsliceHeightPosition;
      oGMsg.info.origin.orientation.x = 0.0;
      oGMsg.info.origin.orientation.y = 0.0;
      oGMsg.info.origin.orientation.z = 0.0;
      oGMsg.info.origin.orientation.w = 1.0;
      oGMsg.data = this->ObstaclesGrid.Flatten();
      this->OccupancyPublisher->publish(oGMsg);
    }
  }

  // Add the points to the map
  // Map is not rolled because if some points are wrong or a frame is bad,
  // the whole map will be rolled and parts will be forgotten.
  // The size of the map is relative to the first frame reveived
  this->DenseMap->Add(preprocessedCloud, false, false);

  // Publish the map
  this->Pointcloud = this->DenseMap->Get(true); // "true" to remove moving objects
  this->Pointcloud->header = registeredCloud->header;
  Pcl2_msg aggregatedCloudMsg;
  pcl::toROSMsg(*this->Pointcloud, aggregatedCloudMsg);
  this->PointsPublisher->publish(aggregatedCloudMsg);
}

//------------------------------------------------------------------------------
void AggregationNode::SavePointcloudService(
    const std::shared_ptr<lidar_slam::srv::SavePc::Request> req,
    const std::shared_ptr<lidar_slam::srv::SavePc::Response> res)
{
  std::string outputPrefix = req->output_prefix_path.empty() ? std::getenv("HOME") : req->output_prefix_path;
  boost::filesystem::path outputPrefixPath(outputPrefix);
  if (!boost::filesystem::exists(outputPrefixPath.parent_path()))
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Output folder does not exist, saving to home folder :" << std::getenv("HOME"));
    outputPrefixPath = boost::filesystem::path(std::getenv("HOME")) / boost::filesystem::path(outputPrefixPath.stem());
  }

  if (req->format > 2 || req->format < 0)
    req->format = 0;
  LidarSlam::PCDFormat f = static_cast<LidarSlam::PCDFormat>(req->format);

  std::string outputFilePath = outputPrefixPath.string() + "_" + std::to_string(int(this->now().seconds())) + ".pcd";
  LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, *this->Pointcloud, f);
  RCLCPP_INFO_STREAM(this->get_logger(), "Pointcloud saved to " << outputFilePath);
  res->success = true;
}

//------------------------------------------------------------------------------
void AggregationNode::ResetService(const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
                                   const std::shared_ptr<lidar_slam::srv::Reset::Response> res)
{
  this->DenseMap->Reset();
  RCLCPP_INFO_STREAM(this->get_logger(), "Resetting aggregation");
  res->success = true;
}

//------------------------------------------------------------------------------
void AggregationNode::PoseCallback(const nav_msgs::msg::Odometry& poseMsg)
{
  this->CurrentPose = Utils::PoseMsgToIsometry(poseMsg.pose.pose);

  if (this->MinDistAroundTrajectory > 0.)
    this->DenseMap->EmptyAroundPoint(this->MinDistAroundTrajectory, this->CurrentPose.translation().cast<float>().array());

  if (!this->DoExtractSlice)
    return;

  // Store pose
  this->Positions.push_back(this->CurrentPose.translation());
  while ((this->CurrentPose.translation() - this->Positions.front()).norm() > this->TrajectoryMaxLength)
    this->Positions.pop_front();

  if (this->Positions.size() < 2)
    return;

  // Extract the slice boundary and compute its area
  // using the last positions and the map
  CloudS boundary;
  double area = this->ExtractSlice(this->SliceWidth, this->SliceMaxDist, this->AngleStep, boundary);

  // Publish the area value
  std_msgs::msg::Float64 areaMsg;
  areaMsg.data = area;
  this->SliceAreaPublisher->publish(areaMsg);

  // Publish the slice points
  Pcl2_msg sliceMsg;
  pcl::toROSMsg(boundary, sliceMsg);
  this->SlicePublisher->publish(sliceMsg);
}

//------------------------------------------------------------------------------
double AggregationNode::ExtractSlice(double sliceWidth, double sliceMaxDist, double angleStep, CloudS& boundary)
{
  // Shortcut to last position
  Eigen::Vector3d& currPosition = this->Positions.back();
  // Deduce the motion direction
  Eigen::Vector3d motionDirection = (currPosition - this->Positions.front()).normalized();
  // Extract the outer voxel in which lays the new pose
  // To reduce the projection tests
  CloudS tmpCloud;
  tmpCloud.resize(1);
  tmpCloud[0].getVector3fMap() = currPosition.cast<float>();
  this->DenseMap->BuildSubMap(tmpCloud, this->MinSlicePtsWithoutMovObjects);
  CloudS::Ptr submap = this->DenseMap->GetSubMap();

  // Initialize the slice pointcloud
  CloudS slice;
  slice.header = this->Pointcloud->header;
  slice.reserve(submap->size());

  // Project them onto the slice plane
  for (const auto& pt : *submap)
  {
    // Check if point in sphere around the trajectory pose
    if ((pt.getVector3fMap().cast<double>() - currPosition).norm() > sliceMaxDist)
      continue;

    double ptDistance = (pt.getVector3fMap().cast<double>() - currPosition).dot(motionDirection);
    // Check if the point is not too far from the trajectory point
    if (std::abs(ptDistance) > sliceWidth)
      continue;

    slice.emplace_back(pt);
    slice.back().getVector3fMap() -= (ptDistance * motionDirection).cast<float>();
  }

  // Create the boundary using a moving average
  // 1. Create a circular histogram
  std::vector<std::vector<Eigen::Vector3d>> histogram(std::ceil((2. * M_PI + 1e-6) / angleStep));
  Eigen::Vector3d refDir = (slice.front().getVector3fMap().cast<double>() - currPosition).normalized();
  for (const auto& pt : slice)
  {
    Eigen::Vector3d currVec = (pt.getVector3fMap().cast<double>() - currPosition).normalized();
    double dotProduct = std::min(std::max(currVec.dot(refDir), -1.), 1.);
    double angle = std::acos(dotProduct);
    if (currVec.cross(refDir).dot(motionDirection) < 0.)
      angle = 2. * M_PI - angle;
    histogram[int(angle / angleStep)].push_back(pt.getVector3fMap().cast<double>());
  }

  // 2. Average bins and fill the boundary cloud
  boundary.header = this->Pointcloud->header;
  boundary.reserve(histogram.size());
  for (const auto& bin : histogram)
  {
    if (bin.empty())
      continue;
    Eigen::Vector3d ptMean = Eigen::Vector3d::Zero();
    for (const auto& pt : bin)
      ptMean += pt;
    ptMean /= bin.size();
    PointS ptBoundary;
    ptBoundary.getVector3fMap() = ptMean.cast<float>();
    boundary.emplace_back(ptBoundary);
  }

  // Browse over the bins to build the area
  double area = 0.;
  for (unsigned int i = 1; i < boundary.size(); ++i)
  {
    Eigen::Vector3d vec1 = boundary[i].getVector3fMap().cast<double>() - currPosition;
    Eigen::Vector3d vec2 = boundary[i - 1].getVector3fMap().cast<double>() - currPosition;
    area += 0.5 * (vec1).cross(vec2).norm();
  }

  return area;
}

//------------------------------------------------------------------------------
void AggregationNode::ExtractZslice(const CloudS& inputCloud, CloudS& zSliceCloud)
{
  // Get plane inliers
  zSliceCloud.header = inputCloud.header;
  CloudS inputCloudInBase;
  pcl::transformPointCloud(inputCloud, inputCloudInBase, this->CurrentPose.inverse().matrix());
  for (int idxPt = 0; idxPt < inputCloud.size(); ++idxPt)
  {
    float distToPlane = std::abs((inputCloudInBase[idxPt].z - this->ZsliceHeightPosition));
    if (!this->InvertZsliceExtraction && distToPlane <= this->ZsliceWidth / 2. ||
        this->InvertZsliceExtraction && distToPlane > this->ZsliceWidth / 2.)
      zSliceCloud.emplace_back(inputCloud[idxPt]);
  }
}

//------------------------------------------------------------------------------
void AggregationNode::LabelObstacle(CloudS& inputCloud, double currentTime)
{
  int unknownLabel = static_cast<int>(LidarSlam::LidarPointLabel::DEFAULT); // shortcut to unknown label

  // Project input potential obstacle points onto grid and
  // fill the current point indices of each pixel
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxPt = 0; idxPt < inputCloud.size(); ++idxPt)
  {
    // Check point is not in a fixed voxel
    if (inputCloud[idxPt].label == static_cast<std::uint16_t>(LidarSlam::LidarPointLabel::FIXED))
      continue;

    // Compute 2D coordinates
    int i = (inputCloud[idxPt].y - this->ObstaclesGrid.Origin.y()) / this->DenseMap->GetLeafSize();
    if (i < 0 || i >= this->ObstaclesGrid.Height)
      continue;
    int j = (inputCloud[idxPt].x - this->ObstaclesGrid.Origin.x()) / this->DenseMap->GetLeafSize();
    if (j < 0 || j >= this->ObstaclesGrid.Width)
      continue;

    // Add point to grid
    this->ObstaclesGrid(i,j).CurrentPtIndices.emplace_back(idxPt);
  }

  // Track number of frames that have seen the pixels
  #pragma omp parallel for num_threads(this->NbThreads)
  for (auto& el : this->ObstaclesGrid.Data)
  {
    Pixel& pix = el.second; // shortcut
    // Notify it has been seen
    if (pix.CurrentPtIndices.size() > 0)
    {
      ++pix.SeenTimes;
      pix.Time = currentTime;
    }
  }

  // Label pixels to clusterize
  #pragma omp parallel for num_threads(this->NbThreads)
  for (auto& el : this->ObstaclesGrid.Data)
  {
    Pixel& pix = el.second; // shortcut
    // Update the cluster label of the pixel
    // if it is not registered and
    // if the pixel has been seen at least once
    if (pix.SeenTimes > 0 &&
        pix.ClusterIdx < 0)
      pix.ClusterIdx = unknownLabel;
  }

  // Link cluster idx to grid indices to speed up search
  std::unordered_map<int, std::vector<int>> clus2gridIndices;
  for (auto& el : this->ObstaclesGrid.Data)
  {
    int pixIdx = el.first; // shortcut
    Pixel& pix = el.second; // shortcut
    if (pix.ClusterIdx < 2)
      continue;
    clus2gridIndices[pix.ClusterIdx].push_back(pixIdx);
  }

  const std::vector<Eigen::Array2i> radii = {{-1, -1}, {0, -1}, {1, -1},
                                             {-1,  0},          {1,  0},
                                             {-1,  1}, {0,  1}, {1,  1}};

  // Grow existing regions
  for (auto& el : clus2gridIndices)
  {
    auto& clusIdx = el.first; // shortcut
    auto& indices = el.second; // shortcut

    std::vector<Eigen::Array2i> clusterPixels;
    clusterPixels.reserve(indices.size());
    // Fill region with existing pixels
    for (int idx : indices)
      clusterPixels.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});

    // Grow region
    int idxPix = 0;
    while (idxPix < clusterPixels.size())
    {
      int i = clusterPixels[idxPix](0);
      int j = clusterPixels[idxPix](1);

      // Label corresponding points
      for (int ptIdx : this->ObstaclesGrid(i, j).CurrentPtIndices)
        inputCloud[ptIdx].label = static_cast<std::uint16_t>(clusIdx);

      // Add neighbors
      #pragma omp parallel for num_threads(this->NbThreads)
      for (const Eigen::Array2i& r : radii)
      {
        Eigen::Array2i neigh = clusterPixels[idxPix] + r;
        // Check neighbor validity
        if (!this->ObstaclesGrid.Check(neigh.x(), neigh.y()))
          continue;

        // If neighbor is occupied, add it to current cluster
        int neighClusIdx = this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx;

        if (neighClusIdx == unknownLabel)
        {
          // Add neighbor to cluster
          clusterPixels.push_back({neigh.x(), neigh.y()});
          // Update label
          this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = clusIdx;
          continue;
        }

        // If the neighbor belongs to another existing cluster
        if (neighClusIdx != clusIdx)
        {
          // Merge the new cluster to the current one
          for (int idx : clus2gridIndices[neighClusIdx])
          {
            // Add to cluster
            clusterPixels.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});
            // Update label
            this->ObstaclesGrid.Data[idx].ClusterIdx = clusIdx;
          }

          // Remove neighbor cluster from clus2gridIndices
          // to not process it afterwards
          clus2gridIndices.erase(neighClusIdx);
        }
      }
      ++idxPix;
    }
  }

  // Fill the potential seeds with remaining unknown pixels
  std::vector<Eigen::Array2i> potentialSeeds;
  #pragma omp parallel for num_threads(this->NbThreads)
  for (auto& el : this->ObstaclesGrid.Data)
  {
    int idx = el.first; // shortcut
    Pixel& pix = el.second; // shortcut
    // Select unknown pixels which are trustworthy enough (they have been seen enough)
    if (pix.ClusterIdx == unknownLabel &&
        pix.SeenTimes >= this->DenseMap->GetMinFramesPerVoxel())
      potentialSeeds.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});
  }

  // Grow new regions
  for (const auto& seed : potentialSeeds)
  {
    // Check seed is still unknown
    if (this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx != unknownLabel)
      continue;

    this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx = this->NewClusterIdx;

    std::vector<Eigen::Array2i> clusterPixels;
    clusterPixels.reserve(this->ObstaclesGrid.Size());
    clusterPixels.emplace_back(seed);
    int idxPix = 0;
    while (idxPix < clusterPixels.size())
    {
      int i = clusterPixels[idxPix](0);
      int j = clusterPixels[idxPix](1);

      // Label corresponding 3D points
      for (int ptIdx : this->ObstaclesGrid(i, j).CurrentPtIndices)
        inputCloud[ptIdx].label = static_cast<std::uint16_t>(this->NewClusterIdx);

      // Check neighbors
      #pragma omp parallel for num_threads(this->NbThreads)
      for (const Eigen::Array2i& r : radii)
      {
        Eigen::Array2i neigh = clusterPixels[idxPix] + r;
        // If neighbor is occupied, add it to current cluster
        if (this->ObstaclesGrid.Check(neigh.x(), neigh.y()) &&
            this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx == unknownLabel)
        {
          // Add neighbor to cluster
          clusterPixels.push_back({neigh.x(), neigh.y()});
          // Update cluster label of pixel
          this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = this->NewClusterIdx;
        }
      }
      ++idxPix;
    }
    ++this->NewClusterIdx;
  }

  // Clear occupancy grid

  // 1. Remove old isolated pixels
  for (auto it = this->ObstaclesGrid.Data.begin(); it != this->ObstaclesGrid.Data.end();)
  {
    Pixel& pix = it->second; // shortcut
    if (pix.ClusterIdx == unknownLabel &&
        currentTime - pix.Time > this->DenseMap->GetDecayingThreshold())
    {
      // Erase and return the iterator to the next element
      it = this->ObstaclesGrid.Data.erase(it);
      continue;
    }
    // Only increment if not erasing
    ++it;
  }

  // 2. Clear pts indices in remaining pixels and reset unknown labels to -1
  #pragma omp parallel for num_threads(this->NbThreads)
  for (auto& el : this->ObstaclesGrid.Data)
  {
    Pixel& pix = el.second; // shortcut

    // Clear current points of each pixel for next fill
    pix.CurrentPtIndices.clear();

    // Reset to -1 the remaining unknown pixels (for visu purposes)
    if (pix.ClusterIdx == unknownLabel)
      pix.ClusterIdx = -1;
  }

  // 3. Fill clus2Time to remove old clusters in 4.
  std::unordered_map<int,double> clus2Time;
  for (auto& el : this->ObstaclesGrid.Data)
  {
    int idx = el.first; // shortcut
    Pixel& pix = el.second; // shortcut

    if (pix.ClusterIdx < 0)
      continue;

    if (!clus2Time.count(pix.ClusterIdx) ||
        pix.Time > clus2Time[pix.ClusterIdx])
      clus2Time[pix.ClusterIdx] = pix.Time;
  }

  // 4. Remove too old clusters
  for (auto& c2t : clus2Time)
  {
    int clusIdx = c2t.first; // shortcut
    double& time = c2t.second; // shortcut
    // Check time
    if (currentTime - time > this->DenseMap->GetDecayingThreshold())
    {
      // Remove old cluster
      for (auto it = this->ObstaclesGrid.Data.begin(); it != this->ObstaclesGrid.Data.end();)
      {
        Pixel& pix = it->second; // shortcut
        if (pix.ClusterIdx == clusIdx)
          // Erase and return the iterator to the next element
          it = this->ObstaclesGrid.Data.erase(it);
        else
          // Only increment if not erasing
          ++it;
      }
    }
  }
}

//-----------------------------------------------------------------------------
void AggregationNode::LoadRefMap(const std::string& path)
{
  if (path.empty())
    return;
  CloudS::Ptr referencePoints(new CloudS);
  if (pcl::io::loadPCDFile(path, *referencePoints) == 0)
  {
    this->RefMap->Add(referencePoints, true, false);
    RCLCPP_INFO_STREAM(this->get_logger(), "Reference map successfully loaded : " << path);

    // Initialize occupancy grid.
    // The grid size and the origin depends on the reference map
    // The resolution is the same as the dense map
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*referencePoints, minPoint, maxPoint);
    this->ObstaclesGrid.Origin = minPoint.head(2);
    Eigen::Vector2f diff = maxPoint.head(2) - minPoint.head(2);
    this->ObstaclesGrid.Height = diff.y() / this->DenseMap->GetLeafSize();
    this->ObstaclesGrid.Width = diff.x() / this->DenseMap->GetLeafSize();
  }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  std::shared_ptr<AggregationNode> aggregationNodePtr =
            std::make_shared<AggregationNode>("aggregation", options);

  // Handle callbacks until shut down
  rclcpp::spin(aggregationNodePtr);

  return 0;
}
