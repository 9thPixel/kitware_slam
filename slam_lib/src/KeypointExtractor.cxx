//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-12-01
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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/KeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <random>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
bool LineFitting::FitLineAndCheckConsistency(const KeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  // Check line width
  float lineLength = (cloud[indices.front()].getVector3fMap() - cloud[indices.back()].getVector3fMap()).norm();
  float widthThreshold = std::max(this->MaxLineWidth, lineLength / this->LengthWidthRatio);

  float maxDist = widthThreshold;
  Eigen::Vector3f bestDirection = Eigen::Vector3f::Zero();
  Eigen::Vector3f bestPosition = Eigen::Vector3f::Zero();

  // RANSAC
  // Sample the indices vector for computation time concerns
  int step = indices.size() > this->MinNbToFit ? indices.size() / this->MinNbToFit : 1;
  std::vector<int> sampledIndices;
  sampledIndices.reserve(indices.size() / step);
  for (int i = 0; i < indices.size(); i += step)
    sampledIndices.emplace_back(indices[i]);

  for (int i = 0; i < sampledIndices.size(); ++i)
  {
    // Extract first point
    auto& point1 = cloud[sampledIndices[i]].getVector3fMap();
    for (int j = i+1; j < sampledIndices.size(); ++j)
    {
      // Extract second point
      auto& point2 = cloud[sampledIndices[j]].getVector3fMap();

      // Compute position of the line (the mean of the two points)
      this->Position = (point1 + point2) / 2.f;

      // Compute line formed by point1 and point2
      this->Direction = (point2 - point1).normalized();

      // Reset score for new points pair
      float currentMaxDist = 0;
      // Compute score : maximum distance of one neighbor to the current line
      for (int idx : sampledIndices)
      {
        currentMaxDist = std::max(currentMaxDist, this->DistanceToPoint(cloud[idx].getVector3fMap()));

        // If the current point distance is too high,
        // the current line won't be selected anyway so we
        // can avoid computing next points' distances
        if (currentMaxDist > widthThreshold)
          break;
      }

      // If the current line implies high error for one neighbor
      // the output line is considered as not trustworthy
      if (currentMaxDist > 2.f * widthThreshold)
        return false;

      if (currentMaxDist <= maxDist)
      {
        bestDirection = this->Direction;
        bestPosition = this->Position;
        maxDist = currentMaxDist;
      }
    }
  }

  if (bestDirection == Eigen::Vector3f::Zero())
    return false;

  this->Direction = bestDirection;
  this->Position = bestPosition;

  return true;
}

//-----------------------------------------------------------------------------
void KeypointExtractor::Enable(const std::vector<Keypoint>& kptTypes)
{
  for (auto& en : this->Enabled)
    en.second = false;
  for (auto& k : kptTypes)
    this->Enabled[k] = true;
}

//-----------------------------------------------------------------------------
bool KeypointExtractor::CheckAzimuthAngle(float azimuthMinRad, float azimuthMaxRad, const Eigen::Vector3f& centralPoint)
{
  if (std::abs(azimuthMaxRad - azimuthMinRad) < 2 * M_PI - 1e-6)
  {
    const float cosAzimuth = centralPoint.x() / std::sqrt(std::pow(centralPoint.x(), 2) + std::pow(centralPoint.y(), 2));
    const float azimuth = centralPoint.y() > 0 ? std::acos(cosAzimuth) : 2 * M_PI - std::acos(cosAzimuth);
    if (azimuthMinRad == azimuthMaxRad)
      return false;
    if (azimuthMinRad < azimuthMaxRad &&
        (azimuth < azimuthMinRad ||
          azimuth > azimuthMaxRad))
      return false;

    if (azimuthMinRad > azimuthMaxRad &&
        (azimuth < azimuthMinRad && azimuth > azimuthMaxRad))
      return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool KeypointExtractor::IsBeamAngleValid(const Eigen::Vector3f& centralPt,
                                         float centralDepth,
                                         const LineFitting& line)
{
  const float beamLineAngle = std::abs(line.Direction.dot(centralPt) / centralDepth);
  return (beamLineAngle <= std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle)));
}
} // End of LidarSlam namespace