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
  //  Get LiDAR id
  this->get_parameter("device_id", this->DeviceId);

  // Get LiDAR spinning speed
  this->get_parameter("rpm", this->Rpm);

  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscribers
  this->ListenerXYZ = this->create_subscription<Pcl2_msg>("/xyz_lidar_points", 1,
                                        std::bind(&RawToLidarNode::CallbackXYZ, this, std::placeholders::_1));

  this->ListenerXYZI = this->create_subscription<Pcl2_msg>("/xyzi_lidar_points", 1,
                                        std::bind(&RawToLidarNode::CallbackXYZI, this, std::placeholders::_1));

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

  CloudS cloudS = Utils::InitCloudS<CloudXYZ>(cloudRaw);

  // Build SLAM pointcloud
  #pragma omp parallel for
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
    slamPoint.device_id = this->DeviceId;
    slamPoint.intensity = 0.;
    slamPoint.laser_id = 0.;
    slamPoint.time = 0.;

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

  CloudS cloudS = Utils::InitCloudS<CloudXYZI>(cloudRaw);

  // Build SLAM pointcloud
  #pragma omp parallel for
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
    slamPoint.device_id = this->DeviceId;
    slamPoint.intensity = rawPoint.intensity;
    slamPoint.laser_id = 0.;
    slamPoint.time = 0.;

    cloudS.push_back(slamPoint);
  }

  PublishMsg(cloudS);
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