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

#pragma once

#include "LidarSlam/State.h"

#include <Eigen/Geometry>
#include <vector>

// Wrapping on vectors of State to make use easier
namespace LidarSlam
{

class StateVector
{
  public:

    StateVector(void);
    StateVector(LidarState startState, LidarState endState);
    StateVector(const std::vector<LidarState>& vecState);
    StateVector& operator=(const StateVector& rhs);
    // Setter
    void SetVec(const std::vector<LidarState>& vecState);
    void SetTransforms(const Eigen::Isometry3d& HStart, const Eigen::Isometry3d& HEnd);
    void SetTimes(double TStart, double TEnd);
    Eigen::Isometry3d GetTransformRange(void) const;
    double GetTimeRange(void) const;

    // Getter
    const std::vector<LidarState>& GetVec(void) const;


  private:
    std::vector<LidarState> VecState;
};

}  // end of LidarSlam namespace