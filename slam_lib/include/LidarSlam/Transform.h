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

#pragma once

#include <Eigen/Geometry>
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{

// Structure to store Lidar slam result
struct Transform
{
//! We use an unaligned Isometry3d in order to avoid having to use
//! Eigen::aligned_allocator<Transform> in each declaration of std::container
//! storing Transform instances, as documented here :
//! http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html

//----------------------------------------------------------------------------
//! Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
Transform(double x, double y, double z, double rx, double ry, double rz);

//! Uses (X, Y, Z, rX, rY, rZ) as input. Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
Transform(const Eigen::Matrix<double, 6, 1>& xyzrpy);

//! Uses roll/pitch/yaw Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy);

Transform(const Eigen::Isometry3d& transform);

Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot);

static Transform Identity() {return Transform(Eigen::Isometry3d::Identity());}

//----------------------------------------------------------------------------
Eigen::UnalignedIsometry3d Isometry = Eigen::UnalignedIsometry3d::Identity();  ///< Isometry representing pose/transform
//----------------------------------------------------------------------------

//! Direct access to translation (position) part.
double& x() {return this->Isometry(0, 3);}
double& y() {return this->Isometry(1, 3);}
double& z() {return this->Isometry(2, 3);}
double x() const {return this->Isometry(0, 3);}
double y() const {return this->Isometry(1, 3);}
double z() const {return this->Isometry(2, 3);}

Eigen::Isometry3d GetIsometry() const { return this->Isometry;}

Eigen::Vector3d GetPosition() const {return this->Isometry.translation();}

Eigen::Translation3d GetTranslation() const {return Eigen::Translation3d(this->Isometry.translation());}

Eigen::Quaterniond GetRotation() const {return Eigen::Quaterniond(this->Isometry.linear());}

Eigen::Matrix4d GetMatrix() const {return this->Isometry.matrix();}
};

} // end of LidarSlam namespace