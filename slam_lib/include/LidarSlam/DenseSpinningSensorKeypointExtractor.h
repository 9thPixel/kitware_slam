//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-10-12
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

#include "Utilities.h"
#include "LidarPoint.h"
#include "Enums.h"
#include "KeypointExtractor.h"
#include <unordered_map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{
class DenseSpinningSensorKeypointExtractor : public KeypointExtractor
{
public:
  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc) override;

  PointCloud::Ptr GetKeypoints(Keypoint k) override;

private:
  // Compute the curvature features within each scan line : depth
  // space gap, intensity gap and line angle
  void ComputeCurvature() override;

  // Auto estimate azimuth angle resolution
  // WARNING: to be correct, the points need to be in the LIDAR sensor
  // coordinates system, where the sensor is spinning around Z axis.
  void EstimateAzimuthalResolution() override;

  // Labelize points (unvalid, edge, plane, blob)
  // and extract them in correspondant pointcloud
  void ComputePlanes() override;
  void ComputeEdges() override;
  void ComputeIntensityEdges() override;
  void ComputeBlobs() override;

  // Add all keypoints of the type k that comply with the threshold criteria for these values
  // The threshold can be a minimum or maximum value (threshIsMax)
  // The weight basis allow to weight the keypoint depending on its certainty
  void AddKptsUsingCriterion (Keypoint k);

  // ---------------------------------------------------------------------------
  //   Parameters specific to the DenseSpinningSensorKeypointExtractor
  // ---------------------------------------------------------------------------

  //TODO

  // ---------------------------------------------------------------------------
  //   Internal variables specific to the DenseSpinningSensorKeypointExtractor
  // ---------------------------------------------------------------------------

  //TODO

};

} // namespace LidarSlam