//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-06
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

#include "LidarSlam/Transform.h"

namespace LidarSlam
{

//------------------------------------------------------------------------------
Transform::Transform(double x, double y, double z, double roll, double pitch, double yaw)
  : Isometry(Utils::XYZRPYtoIsometry(x, y, z, roll, pitch, yaw))
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Vector6d& xyzrpy)
  : Isometry(Utils::XYZRPYtoIsometry(xyzrpy))
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy)
  : Isometry(Utils::XYZRPYtoIsometry(trans(0), trans(1), trans(2), rpy(0), rpy(1), rpy(2)))
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot)
  : Isometry(trans * rot.normalized())
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Isometry3d& isometry)
  : Isometry(isometry)
{}

} // end of LidarSlam namespace