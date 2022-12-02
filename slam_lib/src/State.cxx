//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Authors: Arthur Bourbousson (Kitware SAS),
// Creation date: 2022-10-17
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

#include "LidarSlam/State.h"

namespace LidarSlam
{

PoseStampedVector::PoseStampedVector(void): VecPose{PoseStamped(), PoseStamped()}
{
  VecPose[1].Time = 1.;
}

PoseStampedVector::PoseStampedVector(PoseStamped startPose, PoseStamped endPose): VecPose{startPose, endPose}
{}

PoseStampedVector::PoseStampedVector(const std::vector<PoseStamped>& vecPose): VecPose(vecPose)
{}

PoseStampedVector& PoseStampedVector::operator=(const PoseStampedVector& rhs)
{
  this->VecPose = rhs.VecPose;
  return (*this);
}

// Setter
void PoseStampedVector::SetVec(const std::vector<PoseStamped>& vecPose)
{
  this->VecPose = vecPose;
}

void PoseStampedVector::SetTransforms(const Eigen::Isometry3d& HStart, const Eigen::Isometry3d& HEnd)
{
  this->VecPose.front().Pose = HStart;
  this->VecPose.back().Pose = HEnd;
}

void PoseStampedVector::SetTimes(double TStart, double TEnd)
{
  this->VecPose.front().Time = TStart;
  this->VecPose.back().Time = TEnd;
}

Eigen::Isometry3d PoseStampedVector::GetTransformRange(void) const
{
  return (this->VecPose.front().Pose.inverse() *
          this->VecPose.back().Pose);
}

double PoseStampedVector::GetTimeRange(void) const
{
  return (this->VecPose.back().Time -this->VecPose.front().Time);
}

// Getter
const std::vector<PoseStamped>& PoseStampedVector::GetVec(void) const
{
  return (this->VecPose);
}

} // end of LidarSlam namespace