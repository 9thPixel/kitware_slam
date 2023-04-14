//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Authors: Sanchez Julia (Kitware SAS)
// Creation date: 2023-04-14
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

// This file is necessary to compile PCL
// normal estimation with the Slam point
// see https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html
// for more info 
#include "LidarSlam/LidarPoint.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

PCL_INSTANTIATE_NormalEstimation(LidarSlam::LidarPoint, pcl::Normal)