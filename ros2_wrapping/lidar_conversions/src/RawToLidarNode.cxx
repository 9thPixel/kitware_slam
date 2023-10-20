//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Faure Jeanne (Kitware SAS)
// Creation date: 2023-08-31
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

#include "RawToLidarNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

RawToLidarNode::RawToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
    : Node(node_name, options)
{
  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

  // Get possible frequencies
  this->get_parameter("possible_frequencies", this->PossibleFrequencies);

  // Get number of threads
  this->get_parameter("nb_threads", this->NbThreads);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscribers
  this->ListenerXYZ = this->create_subscription<Pcl2_msg>("/xyz_lidar_points", 1,
                                        std::bind(&RawToLidarNode::CallbackXYZ, this, std::placeholders::_1));

  this->ListenerXYZI = this->create_subscription<Pcl2_msg>("/xyzi_lidar_points", 1,
                                        std::bind(&RawToLidarNode::CallbackXYZI, this, std::placeholders::_1));

  // Init ROS service
  this->EstimService = this->create_service<lidar_conversions::srv::EstimParams>(
      "lidar_conversions/estim_params",
      std::bind(&RawToLidarNode::EstimParamsService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Raw LiDAR data converter is ready !"));
}

//------------------------------------------------------------------------------
void RawToLidarNode::CallbackXYZ(const Pcl2_msg& msg_received)
 {
  CloudXYZ cloudRaw = Utils::InitCloudRaw<CloudXYZ>(msg_received);
  // If input cloud is empty, ignore it
  if (cloudRaw.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input pointcloud is empty : frame ignored.");
    return;
  }

  // Fill the map of device_id if the device hasn't already been attributed one
  if (this->DeviceIdMap.count(cloudRaw.header.frame_id) == 0)
    this->DeviceIdMap[cloudRaw.header.frame_id] = (uint8_t)(this->DeviceIdMap.size());

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudRaw.header.stamp);
  double diffTimePrevFrame = currFrameTime - this->PrevFrameTime;
  this->PrevFrameTime = currFrameTime;

  // If the rotation duration has not been estimated
  if (this->RotationDuration < 0.)
  {
    // Check if this duration is possible
    if (Utils::CheckRotationDuration(diffTimePrevFrame, this->PossibleFrequencies))
    {
      // Check a confirmation of the frame duration to avoid outliers (frames dropped)
      // For the first frame, RotationDurationPrior is -1, this condition won't be fulfilled
      // For the second frame, RotationDurationPrior is absurd, this condition won't be fulfilled
      // First real intempt occurs at the 3rd frame
      if (std::abs(diffTimePrevFrame - this->RotationDurationPrior) < 5e-3) // 5ms threshold
        this->RotationDuration = (diffTimePrevFrame + this->RotationDurationPrior) / 2.;
      this->RotationDurationPrior = diffTimePrevFrame;
      RCLCPP_INFO_STREAM(this->get_logger(), std::setprecision(12) << "Difference between successive frames is :" << diffTimePrevFrame);
    }
  }

  if (this->RotationDuration < 0.)
    return;

  CloudS cloudS = Utils::InitCloudS<CloudXYZ>(cloudRaw);

  const int nbLasers = ((cloudRaw.height >= 8 && cloudRaw.height <= 128)
                   ? static_cast<double>(cloudRaw.height)
                   : (cloudRaw.width >= 8 && cloudRaw.width <= 128)
                     ? static_cast<double>(cloudRaw.width)
                     : this->NbLasers);

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointXYZ>(cloudRaw, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }
  Eigen::Vector2d firstPoint = {cloudRaw[0].x, cloudRaw[0].y};

  uint8_t deviceId = this->DeviceIdMap[cloudRaw.header.frame_id];

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (const PointXYZ& rawPoint : cloudRaw)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rawPoint))
      continue;

    if(!cloudS.empty() && std::equal(rawPoint.data, rawPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;

    slamPoint.x = rawPoint.x;
    slamPoint.y = rawPoint.y;
    slamPoint.z = rawPoint.z;
    slamPoint.device_id = deviceId;
    slamPoint.intensity = 0.;
    slamPoint.laser_id = Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters);
    slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise);

    cloudS.push_back(slamPoint);
  }

  PublishMsg(cloudS);
}

//------------------------------------------------------------------------------
void RawToLidarNode::CallbackXYZI(const Pcl2_msg& msg_received)
{
  CloudXYZI cloudRaw = Utils::InitCloudRaw<CloudXYZI>(msg_received);
  // If input cloud is empty, ignore it
  if (cloudRaw.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input pointcloud is empty : frame ignored.");
    return;
  }

  // Fill the map of device_id if the device hasn't already been attributed one
  if (this->DeviceIdMap.count(cloudRaw.header.frame_id) == 0)
    this->DeviceIdMap[cloudRaw.header.frame_id] = this->DeviceIdMap.size();

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudRaw.header.stamp);
  double diffTimePrevFrame = currFrameTime - this->PrevFrameTime;
  this->PrevFrameTime = currFrameTime;

  // If the rotation duration has not been estimated
  if (this->RotationDuration < 0.)
  {
    // Check if this duration is possible
    if (Utils::CheckRotationDuration(diffTimePrevFrame, this->PossibleFrequencies))
    {
      // Check a confirmation of the frame duration to avoid outliers (frames dropped)
      // For the first frame, RotationDurationPrior is -1, this condition won't be fulfilled
      // For the second frame, RotationDurationPrior is absurd, this condition won't be fulfilled
      // First real intempt occurs at the 3rd frame
      if (std::abs(diffTimePrevFrame - this->RotationDurationPrior) < 5e-3) // 5ms threshold
        this->RotationDuration = (diffTimePrevFrame + this->RotationDurationPrior) / 2.;
      this->RotationDurationPrior = diffTimePrevFrame;
      RCLCPP_INFO_STREAM(this->get_logger(), std::setprecision(12) << "Difference between successive frames is :" << diffTimePrevFrame);
    }
  }

  CloudS cloudS = Utils::InitCloudS<CloudXYZI>(cloudRaw);

  const int nbLasers = ((cloudRaw.height >= 8 && cloudRaw.height <= 128)
                   ? static_cast<double>(cloudRaw.height)
                   : (cloudRaw.width >= 8 && cloudRaw.width <= 128)
                     ? static_cast<double>(cloudRaw.width)
                     : this->NbLasers);

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointXYZI>(cloudRaw, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }
  Eigen::Vector2d firstPoint = {cloudRaw[0].x, cloudRaw[0].y};

  uint8_t deviceId = this->DeviceIdMap[cloudRaw.header.frame_id];

  // Build SLAM pointcloud
  #pragma omp parallel for (this->NbThreads)
  for (const PointXYZI& rawPoint : cloudRaw)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rawPoint))
      continue;

    if(!cloudS.empty() && std::equal(rawPoint.data, rawPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;

    slamPoint.x = rawPoint.x;
    slamPoint.y = rawPoint.y;
    slamPoint.z = rawPoint.z;
    slamPoint.device_id = deviceId;
    slamPoint.intensity = rawPoint.intensity;
    slamPoint.laser_id = Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters);
    slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise);

    cloudS.push_back(slamPoint);
  }
  PublishMsg(cloudS);
}

//------------------------------------------------------------------------------
void RawToLidarNode::EstimParamsService(
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Request> req,
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Response> res)
{
  this->RotSenseAndClustersEstimated = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Estimation parameters will be re-estimated with next frames.");
  res->success = true;
}

}  // namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  std::shared_ptr<lidar_conversions::RawToLidarNode> raw2s
    = std::make_shared<lidar_conversions::RawToLidarNode>("raw_conversion", options);

  rclcpp::spin(raw2s);

  return 0;
}