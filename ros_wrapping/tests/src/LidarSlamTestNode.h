//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <lidar_slam/Confidence.h>
#include <Eigen/Geometry>

namespace Eigen
{
  using Vector6d = Matrix<double, 6, 1>;
}

namespace lidar_slam_test
{

struct Evaluator
{
  double Stamp = 0.;
  float Overlap = 0.f;
  float NbMatches = 0;
  float Duration = 0.f;
};

struct Pose
{
  double Stamp = 0.;
  Eigen::Isometry3d Data = Eigen::Isometry3d::Identity();
};

namespace Utils
{
template<typename T>
class Averaging
{
  public:
  Averaging(const T& initValue) : CurrentValue(initValue) {};

  void Reset() {this->Counter = 0; this->CurrentValue = T();}

  void Update(const T& newValue)
  {
    this->CurrentValue = ((this->CurrentValue * this->Counter) + newValue)/ (this->Counter + 1);
    ++this->Counter;
  }

  T Get() {return this->CurrentValue;}

  Averaging& operator =(const T& value)
  {
    this->CurrentValue = value;
    this->Counter = 0;
    return *this;
  }

  private:
  T CurrentValue;
  int Counter = 0;
};

} // End Utils namespace

/**
 * @class VelodyneToLidarNode aims at converting pointclouds published by ROS
 * Velodyne driver to the expected SLAM pointcloud format.
 *
 * The ROS Velodyne driver can be found here :
 * https://github.com/ros-drivers/velodyne
 */
class LidarSlamTestNode
{

public:
  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  LidarSlamTestNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief Check if comparison with reference data can be performed
   */
  bool CanBeCompared();
  //----------------------------------------------------------------------------
  /*!
   * @brief pose call back, log the data and compare it with reference if required
   * @param pose outputed by lidar slam node
   */
  void PoseCallback(const nav_msgs::Odometry& poseMsg);

  //----------------------------------------------------------------------------
  /*!
   * @brief confidence estimators call back, log the data and compare it with reference if required
   * @param confidence message defined in lidar slam package
   */
  void ConfidenceCallback(const lidar_slam::Confidence& confidence);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber PoseListener;
  ros::Subscriber ConfidenceListener;

  bool Verbose = false;

  // Main boolean to define the success or the failure of the test
  bool Failure = false;

  Eigen::Isometry3d PrevTransform = Eigen::Isometry3d::Identity();
  float LastAngleDiff = 0.f;
  float LastPositionDiff = 0.f;

  // Path to the folder where to store the results (folder must exist)
  std::string ResPath = "/tmp";

  // Comparison

  // Path to the folder containing the reference results to compare with
  // If empty, no comparison is performed
  std::string RefPath;

  // Storage for reference data (loaded from RefPath)
  std::vector<Evaluator> RefEvaluators;
  std::vector<Pose> RefPoses;

  // Storage for current results

  // Parameters to keep track of the pose index to compare with reference
  unsigned int PoseIdx = 0;
  unsigned int PrevPoseIdx = 0;
  // Counter to keep track of the confidence index to compare with reference
  unsigned int ConfidenceCounter = 0;
  // To count number of dropped frames with respect to reference
  unsigned int NbFramesDropped = 0;

  // Global evaluator for the whole trajectory
  Utils::Averaging<float> DiffAngle = 0.f;
  Utils::Averaging<float> DiffPosition = 0.f;
  Utils::Averaging<float> DiffOverlap = 0.f;
  Utils::Averaging<float> DiffTime = 0.f;
  Utils::Averaging<float> DiffNbMatches = 0.f;

  // Thresholds to warn the user :
  float PositionThreshold = 0.2f;  // 20cm
  float AngleThreshold    = 5.f;   // 5°
  float TimeThreshold     = 0.01f; // 10ms
  int MaxNbFramesDropped  = 5;     // 5 frames

private:

  void LoadRef();
  void OutputTestResult();
};

}  // end of namespace lidar_conversions
