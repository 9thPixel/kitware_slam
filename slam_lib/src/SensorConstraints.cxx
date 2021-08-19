//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2021-04-02
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

#include "LidarSlam/SensorConstraints.h"

namespace LidarSlam
{
namespace SensorConstraints
{

void WheelOdometryManager::ComputeWheelAbsoluteConstraint(double lidarTime)
{
  // Protect Measures data (reading) from outside modification
  // (Adding a measure with external thread)
  std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);

  this->ResetResidual();

  // Check if weight is not null and measures contains elements
  if (!this->CanBeUsed())
    return;

  // If sensor times don't match,
  // Do not add constraint to optimization
  lidarTime -= this->TimeOffset;
  if (lidarTime < this->Measures.front().Time || lidarTime >= this->Measures.back().Time)
  {
    PRINT_WARNING("No odometry measure corresponds to the current frame acquisition (times don't match) : "
               << "no absolute wheel odometric constraint added to optimization");
    return;
  }

  // Reset if it is the first call or if the timeline has been modified
  if (!this->IsRef || this->PrevIt->Time > lidarTime)
    this->PrevIt = this->Measures.begin();

  // Get index of first odometry measurement after LiDAR time
  auto nextIt = this->PrevIt;
  while (nextIt->Time <= lidarTime)
    ++nextIt;
  // Get index of last odometry measurement before LiDAR time
  auto prevIt = nextIt;
  --prevIt;
  // Interpolate odometry measurement at LiDAR timestamp (between prevIt and nextIt measures)
  float rt = (lidarTime - prevIt->Time) / (nextIt->Time - prevIt->Time);
  float currDistance = (1 - rt) * prevIt->Distance + rt * nextIt->Distance;

  // Build odometry residual

  // If this is the first pose,
  // initialize reference with the current first data
  if (this->IsRef)
  {
    std::cout << "No wheel odometry measure corresponding to previous pose : "
              << "no absolute wheel odometric constraint added to optimization" << std::endl;
    // Update index and distance for next frame
    this->PrevIt = prevIt;
    this->RefDistance = currDistance;
    this->IsRef = true;
    return;
  }

  // If the reference has been computed,
  // add the corresponding constraint
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->RefPose.translation(), currDistance - this->RefDistance);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  std::cout << "Adding wheel odometry residual : " << currDistance - this->RefDistance << " m travelled since first frame." << std::endl;

  // Update current iterator for next frame
  this->PrevIt = prevIt;
}

void WheelOdometryManager::ComputeWheelOdomConstraint(double lidarTime)
{
  // Protect Measures data (reading) from outside modification
  // (Adding a measure with external thread)
  std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);

  this->ResetResidual();

  if (!this->CanBeUsed())
    return;

  // If sensor times don't match,
  // Do not add constraint to optimization
  // Reset IsRef to recompute the reference odometry value in next call
  lidarTime -= this->TimeOffset;
  if (lidarTime < this->Measures.front().Time || lidarTime >= this->Measures.back().Time)
  {
    PRINT_WARNING("No odometry measure corresponds to the current frame acquisition (times don't match) : "
               << "no relative wheel odometric constraint added to optimization");
    this->IsRef = false;
    return;
  }

  // If the timeline has been modified, reference odometry value needs to be updated
  if (this->PrevIt->Time > lidarTime)
    this->IsRef = false;

  // Reset if the reference pose does not exist
  if (!this->IsRef)
    this->PrevIt = this->Measures.begin();

  // Get index of first odometry measurement after LiDAR time
  auto nextIt = this->PrevIt;
  while (nextIt->Time <= lidarTime)
    ++nextIt;
  // Get index of last odometry measurement before LiDAR time
  auto prevIt = nextIt;
  --prevIt;
  // Interpolate odometry measurement at LiDAR timestamp (between prevIt and nextIt measures)
  float rt = (lidarTime - prevIt->Time) / (nextIt->Time - prevIt->Time);
  float currDistance = (1 - rt) * prevIt->Distance + rt * nextIt->Distance;

  // If the reference pose has been set,
  // build constraint with relative spatial distance between
  // the two successive poses
  if (this->IsRef)
  {
    float distDiff = std::abs(currDistance - this->RefDistance);
    this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->RefPose.translation(), distDiff);
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
    std::cout << "Adding relative wheel odometry residual : " << distDiff << " m travelled since last frame." << std::endl;
  }
  // If the reference pose has not been set (e.g. first call / reset / time unconsistency),
  // no constraint is added
  else
    std::cout << "No previous wheel odometry measure : no relative wheel odometric constraint added to optimization" << std::endl;

  // Update current iterator for next frame
  this->PrevIt = prevIt;
  // Update reference distance for next frame
  this->RefDistance = currDistance;
  // Update manager state
  this->IsRef = true;
}

void ImuManager::ComputeGravityConstraint(double lidarTime)
{
  // Protect Measures data (reading) from outside modification
  // (Adding a measure with external thread)
  std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);

  this->ResetResidual();

  if (!this->CanBeUsed())
    return;

  // If sensor times don't match,
  // Do not add constraint to optimization
  lidarTime -= this->TimeOffset;
  if (lidarTime < this->Measures.front().Time || lidarTime >= this->Measures.back().Time)
  {
    PRINT_WARNING("No IMU measure corresponds to the current frame acquisition (times don't match) : "
               << "no IMU constraint added to optimization");
    return;
  }

  // Compute reference gravity vector if needed
  if (this->GravityRef.norm() < 1e-6)
    this->ComputeGravityRef(Utils::Deg2Rad(5.f));

  // Reset if it is the first call or if the timeline has been modified
  if (!this->IsRef || this->PrevIt->Time > lidarTime)
    this->PrevIt = this->Measures.begin();

  // Get index of first odometry measurement after LiDAR time
  auto nextIt = this->PrevIt;
  while (nextIt->Time <= lidarTime)
    ++nextIt;
  // Get index of last odometry measurement before LiDAR time
  auto prevIt = nextIt;
  --prevIt;
  // Interpolate gravity measurement at LiDAR timestamp (between prevIt and nextIt measures)
  float rt = (lidarTime - prevIt->Time) / (nextIt->Time - prevIt->Time);
  Eigen::Vector3d gravityDirection = (1 - rt) * prevIt->Acceleration.normalized() + rt * nextIt->Acceleration.normalized();

  // Normalize interpolated gravity vector
  if (gravityDirection.norm() > 1e-6) // Check to insure consistent IMU measure
    gravityDirection.normalize();
  else
  {
    PRINT_WARNING("Gravity constraint could not be computed : no IMU constraint added to optimization");
    return;
  }

  // Build gravity constraint
  this->Residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(this->GravityRef, gravityDirection);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));

  // Update current iterator for next frame
  this->PrevIt = prevIt;
}

void ImuManager::ComputeGravityRef(double deltaAngle)
{
  // Init histogram 2D (phi and theta)
  int NPhi = std::ceil(2 * M_PI / deltaAngle);
  int NTheta = std::ceil(M_PI / deltaAngle);
  std::vector<std::vector<std::vector<int>>> histogram(NPhi, std::vector<std::vector<int>>(NTheta));

  // Store acceleration vector indices in histogram
  for (unsigned int idxAcc = 0; idxAcc < this->Measures.size(); ++idxAcc)
  {
    Eigen::Vector3d AccelDirection = this->Measures[idxAcc].Acceleration.normalized();
    int idxPhi = ( std::atan2(AccelDirection.y(), AccelDirection.x()) + M_PI ) / deltaAngle;
    int idxTheta = ( std::acos(AccelDirection.z()) ) / deltaAngle;
    histogram[idxPhi][idxTheta].push_back(idxAcc);
  }
  // Get bin containing most points
  int bestPhi = 0;
  int bestTheta = 0;
  for (int idxPhi = 0; idxPhi < NPhi; ++idxPhi)
  {
    for (int idxTheta = 0; idxTheta < NTheta; ++idxTheta)
    {
      if (histogram[idxPhi][idxTheta].size() > histogram[bestPhi][bestTheta].size())
      {
        bestPhi = idxPhi;
        bestTheta = idxTheta;
      }
    }
  }

  // Compute mean of acceleration vectors in this bin 
  this->GravityRef = Eigen::Vector3d::Zero();
  for (int idxAcc : histogram[bestPhi][bestTheta])
    this->GravityRef += this->Measures[idxAcc].Acceleration.normalized();
  this->GravityRef.normalize();
  std::cout << "Gravity vector : " << this->GravityRef.transpose() << std::endl;
}

} // end of SensorConstraints namespace
} // end of LidarSlam namespace