//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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
#include "LidarSlam/SpinningSensorKeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace LidarSlam
{
//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->pclCurrentFrame = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Invalid points with bad criteria
  this->InvalidPointWithBadCriteria();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->pclCurrentFrameByScan)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as pclCurrentFrameByScan is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->pclCurrentFrame)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->pclCurrentFrameByScan.size())
      this->pclCurrentFrameByScan.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->pclCurrentFrameByScan[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NLasers = this->pclCurrentFrameByScan.size();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Do not use clear(), otherwise weird things could happen if outer program
  // uses these pointers
  this->EdgesPoints.reset(new PointCloud);
  this->PlanarsPoints.reset(new PointCloud);
  this->BlobsPoints.reset(new PointCloud);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->EdgesPoints);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->PlanarsPoints);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->BlobsPoints);

  // Initialize the features vectors with the correct length
  this->Curvature.resize(this->NLasers);
  this->IntensityGrad.resize(this->NLasers);
  this->IsPointValid.resize(this->NLasers);
  this->Label.resize(this->NLasers);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    size_t nbPoint = this->pclCurrentFrameByScan[scanLine]->size();
    this->IsPointValid[scanLine].assign(nbPoint, KeypointFlags().set());  // set all flags to 1
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Curvature[scanLine].assign(nbPoint, 0.);
    this->IntensityGrad[scanLine].assign(nbPoint, 0.);
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidPointWithBadCriteria()
{
  const float expectedLengthCoeff = 10.;

  // loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) firstprivate(expectedLengthCoeff)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      for (int index = 0; index < Npts; ++index)
        this->IsPointValid[scanLine][index].reset();
      continue;
    }

    // invalidate first and last points
    // CHECK why ?
    for (int index = 0; index <= this->NeighborWidth; ++index)
      this->IsPointValid[scanLine][index].reset();
    for (int index = Npts - 1 - this->NeighborWidth - 1; index < Npts; ++index)
      this->IsPointValid[scanLine][index].reset();

    // loop over points into the scan line
    for (int index = this->NeighborWidth; index < Npts - this->NeighborWidth - 1; ++index)
    {
      // double precision is useless as PCL points coordinates are internally stored as float
      const Eigen::Vector3f& previousPoint = scanLineCloud[index - 1].getVector3fMap();
      const Eigen::Vector3f& currentPoint  = scanLineCloud[index    ].getVector3fMap();
      const Eigen::Vector3f& nextPoint     = scanLineCloud[index + 1].getVector3fMap();

      const float L = currentPoint.norm();
      const float Ln = nextPoint.norm();
      const float dLn = (nextPoint - currentPoint).norm();
      const float dLp = (currentPoint - previousPoint).norm();

      // The expected length between two firings of the same laser is the
      // distance along the same circular arc. It depends only on radius value
      // and the angular resolution of the sensor.
      // We multiply this length by a coeff for tolerance.
      const float expectedLength = this->AngleResolution * L * expectedLengthCoeff;
      const float sqExpectedLength = expectedLength * expectedLength;

      // Invalid occluded points due to depth gap.
      // If the distance between two successive points is bigger than the
      // expected length, it means that there is a depth gap.
      if (dLn > expectedLength)
      {
        // We must invalidate the points which belong to the occluded area (farthest).
        // If current point is the closest, invalid next part, starting from next point
        if (L < Ln)
        {
          this->IsPointValid[scanLine][index + 1].reset();
          for (int i = index + 2; i <= index + this->NeighborWidth; ++i)
          {
            const Eigen::Vector3f& Y  = scanLineCloud[i - 1].getVector3fMap();
            const Eigen::Vector3f& Yn = scanLineCloud[i].getVector3fMap();

            // If there is a gap in the neighborhood, we do not invalidate the rest of it.
            if ((Yn - Y).squaredNorm() > sqExpectedLength)
            {
              break;
            }
            // Otherwise, do not use next point
            this->IsPointValid[scanLine][i].reset();
          }
        }
        // If current point is the farest, invalid previous part, starting from current point
        else
        {
          this->IsPointValid[scanLine][index].reset();
          for (int i = index - this->NeighborWidth; i < index; ++i)
          {
            const Eigen::Vector3f& Yp = scanLineCloud[i].getVector3fMap();
            const Eigen::Vector3f&  Y = scanLineCloud[i + 1].getVector3fMap();

            // If there is a gap in the neighborhood, we do not invalidate the rest of it.
            if ((Y - Yp).squaredNorm() > sqExpectedLength)
            {
              break;
            }
            // Otherwise, do not use previous point
            this->IsPointValid[scanLine][i].reset();
          }
        }
      }

      // Invalid points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index].reset();
      }

      // Invalid points which are on a planar surface nearly parallel to the
      // laser beam direction
      else if ((dLp > 0.25 * expectedLength) &&
               (dLn > 0.25 * expectedLength))
      {
        this->IsPointValid[scanLine][index].reset();
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (this->IsPointValid[scanLine][index].none())
      {
        continue;
      }

      // 1_ Compute intensity gradient
      // CHECK : do not use currentPoint.intensity?
      const Point& previousPoint = scanLineCloud[index - 1];
      const Point& nextPoint     = scanLineCloud[index + 1];
      this->IntensityGrad[scanLine][index] = std::abs(nextPoint.intensity - previousPoint.intensity);

      // 2_ Estimate curvature
      // Init curvature value
      this->Curvature[scanLine][index] = 0.f;
      // Apply kernel [1 1 ... 1 1 -N 1 1 ... 1 1 ] (size N + 1) to neighborhood depths
      // Loop over neighbors
      for (int n = index - this->NeighborWidth; n <= index + this->NeighborWidth; ++n)
        this->Curvature[scanLine][index] += scanLineCloud[n].getVector3fMap().norm();
      this->Curvature[scanLine][index] -= (2 * this->NeighborWidth + 1) * scanLineCloud[index].getVector3fMap().norm();
      this->Curvature[scanLine][index] = std::abs(this->Curvature[scanLine][index]);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  // Loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    const int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // Sort the curvature/intensity scores in a decreasing order
    std::vector<size_t> sortedCurvatureIdx     = Utils::SortIdx(this->Curvature[scanLine]);
    std::vector<size_t> sortedIntensityGradIdx = Utils::SortIdx(this->IntensityGrad[scanLine]);

    // 1_ Set edge keypoints labels
    auto labelEdgesUsingCriterion = [this, scanLine, Npts](const std::vector<size_t>& sortedValuesIdx,
                                                         const std::vector<std::vector<float>>& values,
                                                         float threshold,
                                                         int invalidNeighborhoodSize)
    {
      for (const auto& index: sortedValuesIdx)
      {
        // Check criterion threshold
        // If criterion is not respected, break loop as indices are sorted in decreasing order.
        if (values[scanLine][index] < threshold)
          break;

        // If the point is invalid as edge, continue
        if (!this->IsPointValid[scanLine][index][Keypoint::EDGE])
          continue;

        // Else indicate that the point is an edge
        this->Label[scanLine][index].set(Keypoint::EDGE);

        // Invalidate the point and its neighbors for EDGE search
        const int indexBegin = std::max(0,        static_cast<int>(index - invalidNeighborhoodSize));
        const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + invalidNeighborhoodSize));
        for (int j = indexBegin; j <= indexEnd; ++j)
          this->IsPointValid[scanLine][j].reset(Keypoint::EDGE);
        // Invalidate the point for all future keypoints type search
        this->IsPointValid[scanLine][index].reset();
      }
    };

    // Extract edge keypoints using curvature
    labelEdgesUsingCriterion(sortedCurvatureIdx, this->Curvature, this->EdgeCurvatureThreshold, this->NeighborWidth);
    // Extract edge keypoints using intensity
    labelEdgesUsingCriterion(sortedIntensityGradIdx, this->IntensityGrad, this->EdgeIntensityGradThreshold, 1);
    
    // 2_ Set plane keypoints label
    for (int i = int(Npts - 1); i>= 0; --i)
    {
      int index = sortedCurvatureIdx[i];
      // Check criterion threshold
      // If criterion is not respected, break loop as indices are analysed in value increasing order.
      if (this->Curvature[scanLine][index] > this->PlaneCurvatureThreshold)
        break;

      // if the point is invalid as plane, continue
      if (!this->IsPointValid[scanLine][index][Keypoint::PLANE])
        continue;

      // else indicate that the point is a planar one
      this->Label[scanLine][index].set(Keypoint::PLANE);

      // Invalidate the point and its neighbors for PLANE search
      const int indexBegin = std::max(0,        static_cast<int>(index - this->NeighborWidth));
      const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + this->NeighborWidth));
      for (int j = indexBegin; j <= indexEnd; ++j)
        this->IsPointValid[scanLine][j].reset(Keypoint::PLANE);
      // Invalidate the point for all future keypoints type search
      this->IsPointValid[scanLine][index].reset();
    }

    // 3_ Set blob keypoints labels
    for (int index = 0; index < Npts; ++index)
    {
      // If the point is invalid as blob, continue
      if (!this->IsPointValid[scanLine][index][Keypoint::BLOB])
        continue;

      // Else indicate that the point is a blob
      this->Label[scanLine][index].set(Keypoint::BLOB);

      // Invalidate the point and its neighbors for BLOB search
      const int indexBegin = std::max(0,        static_cast<int>(index - this->NeighborWidth));
      const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + this->NeighborWidth));
      for (int j = indexBegin; j <= indexEnd; ++j)
        this->IsPointValid[scanLine][j].reset(Keypoint::BLOB);
      // Invalidate the point for all future keypoints type search
      this->IsPointValid[scanLine][index].reset();
    }
  }

  auto addKeypoints = [this](Keypoint type, PointCloud::Ptr& keypoints)
  {
    for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
    {
      const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
      for (unsigned int index = 0; index < scanLineCloud.size(); ++index)
      {
        if (this->Label[scanLine][index][type])
          keypoints->push_back(scanLineCloud[index]);
      }
    }
  };
  addKeypoints(Keypoint::EDGE, this->EdgesPoints);
  addKeypoints(Keypoint::PLANE, this->PlanarsPoints);
  addKeypoints(Keypoint::BLOB, this->BlobsPoints);
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->pclCurrentFrame->size());
    std::vector<int> indexByScanLine(this->NLasers, 0);
    for (unsigned int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      const auto& laserId = this->pclCurrentFrame->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->pclCurrentFrame->size());
    std::vector<int> indexByScanLine(this->NLasers, 0);
    for (unsigned int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      const auto& laserId = this->pclCurrentFrame->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["curvature"]  = get1DVector(this->Curvature);
  map["intensity_grad"]  = get1DVector(this->IntensityGrad);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["edge_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::EDGE);
  map["plane_validity"] = get1DVectorFromFlag(this->IsPointValid, Keypoint::PLANE);
  map["blob_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::BLOB);
  return map;
}

} // end of LidarSlam namespace