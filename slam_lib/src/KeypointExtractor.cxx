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


namespace LidarSlam
{

//-----------------------------------------------------------------------------
bool LineFitting::FitLineAndCheckConsistency(const KeypointExtractor::PointCloud& cloud,
                                             const std::vector<int>& indices)
{
  // Check consistency of the line
  if (indices.size() < 2)
    return false;

  float distMax = 0.f;
  float distMin = std::numeric_limits<float>::max();
  float distCurr;
  for (int i = 1; i < indices.size(); ++i)
  {
    distCurr = (cloud[indices[i]].getVector3fMap() - cloud[indices[i-1]].getVector3fMap()).squaredNorm();
    distMax = std::max(distMax, distCurr);
    distMin = std::min(distMin, distCurr);
  }
  if (distMax / distMin > this->SquaredRatio)
    return false;

  Eigen::Vector3f diffVec = cloud[indices.back()].getVector3fMap() - cloud[indices.front()].getVector3fMap();
  this->Direction = diffVec.normalized();
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, indices, centroid);
  this->Position = centroid.head(3);

  // Check line width
  float lineLength = diffVec.norm();
  float widthThreshold = std::max(this->MaxLineWidth, lineLength / this->LengthWidthRatio);

  for (auto idx : indices)
  {
    float error = (cloud[idx].getVector3fMap() - this->Position).cross(this->Direction).norm();
    if (error > widthThreshold)
      return false;
  }

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