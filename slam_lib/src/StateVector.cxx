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

#include "LidarSlam/StateVector.h"

namespace LidarSlam
{

StateVector::StateVector(void): VecState{LidarState(), LidarState()}
{
  VecState[1].Time = 1.;
}

StateVector::StateVector(LidarState startState, LidarState endState): VecState{startState, endState}
{}

StateVector::StateVector(const std::vector<LidarState>& vecState): VecState(vecState)
{}

StateVector& StateVector::operator=(const StateVector& rhs)
{
  this->VecState = rhs.VecState;
  return (*this);
}


// Setter
void StateVector::SetVec(const std::vector<LidarState>& vecState)
{
  this->VecState = vecState;
}

void StateVector::SetTransforms(const Eigen::Isometry3d& HStart, const Eigen::Isometry3d& HEnd)
{
  this->VecState.front().Isometry = HStart;
  this->VecState.back().Isometry = HEnd;
}

void StateVector::SetTimes(double TStart, double TEnd)
{
  this->VecState.front().Time = TStart;
  this->VecState.back().Time = TEnd;
}

Eigen::Isometry3d StateVector::GetTransformRange(void) const
{
  return (this->VecState.front().Isometry.inverse() *
          this->VecState.back().Isometry);
}

double StateVector::GetTimeRange(void) const
{
  return (this->VecState.back().Time -this->VecState.front().Time);
}

// Getter
const std::vector<LidarState>& StateVector::GetVec(void) const
{
  return (this->VecState);
}

} // end of LidarSlam namespace