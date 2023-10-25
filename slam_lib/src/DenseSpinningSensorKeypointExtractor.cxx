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

#include "LidarSlam/DenseSpinningSensorKeypointExtractor.h"
#include "LidarSlam/Utilities.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <random>

#include <iostream>
#include <fstream>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
DenseSpinningSensorKeypointExtractor::PointCloud::Ptr DenseSpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  //TODO
}

//-----------------------------------------------------------------------------
std::shared_ptr<PtFeat> DenseSpinningSensorKeypointExtractor::GetPtFeat(int idxInScan)
{
  auto& indices = this->Pc2VmIndices[idxInScan];
  return this->VertexMap[indices.Row][indices.Col];
}

//-----------------------------------------------------------------------------
int DenseSpinningSensorKeypointExtractor::GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine)
{
  int nbPoints;
  for (auto& point : scanLine)
  {
    if (point != nullptr)
      nbPoints++;
  }
  return nbPoints;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  std::vector<LidarPoint> prevPtByLine(this->NbLaserRings, this->Scan->at(0));
  for (const LidarPoint& point : *this->Scan)
  {
    auto& prevPt = prevPtByLine[this->LaserIdMap.find(point.laser_id)->second];
    Eigen::Map<const Eigen::Vector2f> pt1(point.data);
    Eigen::Map<const Eigen::Vector2f> pt2(prevPt.data);
    float angle = std::abs(std::acos(pt1.dot(pt2) / (pt1.norm() * pt2.norm())));

    // Keep only angles greater than 0 to avoid dual return issues
    if (angle > 1e-4)
      angles.push_back(angle);

    prevPt = point;
  }

  // A minimum number of angles is needed to get a trustable estimator
  if (angles.size() < 100)
  {
    PRINT_WARNING("Not enough points to estimate azimuthal resolution");
    return;
  }

  // Estimate azimuthal resolution from these angles
  std::sort(angles.begin(), angles.end());
  unsigned int maxInliersIdx = angles.size();
  float maxAngle = Utils::Deg2Rad(5.);
  float medianAngle = 0.;
  // Iterate until only angles between direct LiDAR beam neighbors remain.
  // The max resolution angle is decreased at each iteration.
  while (maxAngle > 1.8 * medianAngle)
  {
    maxInliersIdx = std::upper_bound(angles.begin(), angles.begin() + maxInliersIdx, maxAngle) - angles.begin();
    medianAngle = angles[maxInliersIdx / 2];
    maxAngle = std::min(medianAngle * 2., maxAngle / 1.8);
  }
  this->AzimuthalResolution = medianAngle;
  std::cout << "LiDAR's azimuthal resolution estimated to " << Utils::Rad2Deg(this->AzimuthalResolution) << "°" << std::endl;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::InitInternalParameters()
{
  // Map laser_ids
  for (const auto& point : *this->Scan)
  {
    if (this->LaserIdMap.count(point.laser_id) == 0)
      this->LaserIdMap[point.laser_id] = this->LaserIdMap.size();
  }

  // Save the number of lasers
  this->NbLaserRings = this->LaserIdMap.size();

  this->RotationIsClockwise = Utils::IsRotationClockwise<Point>(*this->Scan, this->NbLaserRings);

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();

  // Compute the indices of scan points in the future vertex map
  this->Pc2VmIndices.reserve(this->Scan->size());
  for (const auto& point : *this->Scan)
  {
    float azimuth = this->RotationIsClockwise ? M_PI + std::atan2(point.y, point.x) : M_PI - std::atan2(point.y, point.x);
    this->Pc2VmIndices.emplace_back(IdxVM{this->LaserIdMap.find(point.laser_id)->second,
                                          static_cast<int>(azimuth / this->AzimuthalResolution)});
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::CreateVertexMap()
{
  this->WidthVM = 2. * M_PI / this->AzimuthalResolution + 1;
  this->HeightVM = this->NbLaserRings;

  this->VertexMap.resize(this->HeightVM, std::vector<std::shared_ptr<PtFeat>>(this->WidthVM));
  // Fill Vertex Map with index and depth values
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->Scan->size(); ++i)
  {
    const Point& point = this->Scan->at(i);
    auto& indices = this->Pc2VmIndices[i];

    this->VertexMap[indices.Row][indices.Col] = std::make_shared<PtFeat>();
    this->VertexMap[indices.Row][indices.Col]->Index = i;
    this->VertexMap[indices.Row][indices.Col]->Depth = point.getVector3fMap().norm();
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ClearVertexMap()
{
  for (auto& row : this->VertexMap)
  {
    for (auto& ptr : row)
      ptr.reset(); // ptr = nullptr
  }

  this->Pc2VmIndices.clear();
}

//-----------------------------------------------------------------------------
#define OUTPUT_FEATURE(filename, featName, maxFeat)                               \
do                                                                                \
{                                                                                 \
  int nbColors = std::max(5, std::min(255,                                        \
                  static_cast<int>(maxFeat < 255.f ? maxFeat + 1.f : 255.f)));    \
  std::ofstream file(filename);                                                   \
  file << "P2\n";                                                                 \
  file << this->WidthVM << " " << this->HeightVM << "\n";                         \
  file << nbColors << "\n";                                                       \
  for (int i = 0; i < this->HeightVM; ++i)                                        \
  {                                                                               \
    for (int j = 0; j < this->WidthVM; ++j)                                       \
    {                                                                             \
      int value;                                                                  \
      if (this->VertexMap[i][j] == nullptr)                                       \
        value = 0;                                                                \
      else                                                                        \
      {                                                                           \
        float featValue = this->VertexMap[i][j]->featName;                        \
        if (featValue >= 0.f)                                                     \
        {                                                                         \
          float scaledValue = std::min(static_cast<float>(maxFeat), featValue);   \
          value = static_cast<int>((scaledValue / maxFeat) * (nbColors - 2)) + 1; \
        }                                                                         \
        else                                                                      \
          value = 1;                                                              \
      }                                                                           \
      file << value << " ";                                                       \
    }                                                                             \
    file << "\n";                                                                 \
  }                                                                               \
  file.close();                                                                   \
} while (0)

void DenseSpinningSensorKeypointExtractor::OutputFeatures()
{
  int totalSize = this->WidthVM * this->HeightVM;
  OUTPUT_FEATURE("/tmp/Index.pgm", Index, totalSize);
  OUTPUT_FEATURE("/tmp/Depth.pgm", Depth, 20.);
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  this->InitInternalParameters();

  this->CreateVertexMap();

  this->ClearVertexMap();
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeCurvature()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKptsUsingCriterion(Keypoint k)
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputePlanes()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeEdges()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  //TODO
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeBlobs()
{
  //TODO
}
} // end of LidarSlam namespace