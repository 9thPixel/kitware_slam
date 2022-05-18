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

#include <pcl/common/common.h>

namespace LidarSlam
{

namespace
{
//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  // Compute PCA to determine best line approximation of the points distribution
  // and save points centroid in Position
  Eigen::Vector3f eigVals;
  Eigen::Matrix3f eigVecs;
  Utils::ComputeMeanAndPCA(cloud, indices, this->Position, eigVecs, eigVals);

  // Get Direction as main eigen vector
  this->Direction = eigVecs.col(2);

  // Compute line features
  float LineLength = (cloud[indices[0]].getVector3fMap() - cloud[indices[indices.size() - 1]].getVector3fMap()).norm();
  float LineWidth = 0;
  for (int idx: indices)
    LineWidth = std::max(LineWidth, this->DistanceToPoint(cloud[idx].getVector3fMap()));


  // If the line is too wide (MSE too high) respectively to its length,
  // we consider the neighborhood as not flat enough
  // If the line is too short, it is considered not trutworthy enough
  if (LineWidth > this->MaxLineWidth || LineLength < this->MinLineLength)
    return false;

  return true;
}

//-----------------------------------------------------------------------------
inline float LineFitting::DistanceToPoint(Eigen::Vector3f const& point) const
{
  return ((point - this->Position).cross(this->Direction)).norm();
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
float SpinningSensorKeypointExtractor::GetVoxelResolution() const
{
  if (this->Keypoints.empty())
    return -1.;
  return this->Keypoints.begin()->second.GetVoxelResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetVoxelResolution(float res)
{
  for (auto& kptsVG : this->Keypoints)
    kptsVG.second.SetVoxelResolution(res);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::Enable(const std::vector<Keypoint>& kptTypes)
{
  for (auto& en : this->Enabled)
    en.second = false;
  for (auto& k : kptTypes)
    this->Enabled[k] = true;
}

//-----------------------------------------------------------------------------
SpinningSensorKeypointExtractor::PointCloud::Ptr SpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypoints = this->Keypoints.at(k).GetCloud(this->MaxPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypoints);
  return keypoints;
}


//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Invalidate points with bad criteria
  this->InvalidateNotUsablePoints();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize and extract keypoints
  // Warning : order matters
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->ScanLines)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as ScanLines is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->Scan)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->ScanLines.size())
      this->ScanLines.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->ScanLines[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NbLaserRings = this->ScanLines.size();

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NbLaserRings);
  this->DepthGap.resize(this->NbLaserRings);
  this->IntensityGap.resize(this->NbLaserRings);
  this->IsPointValid.resize(this->NbLaserRings);
  this->Label.resize(this->NbLaserRings);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    size_t nbPoint = this->ScanLines[scanLine]->size();
    this->IsPointValid[scanLine].assign(nbPoint, true);  // set all points as valid
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLine].assign(nbPoint, -1.);
    this->DepthGap[scanLine].assign(nbPoint, -1.);
    this->IntensityGap[scanLine].assign(nbPoint, -1.);
  }

  // Reset voxel grids
  LidarPoint minPt, maxPt;
  pcl::getMinMax3D(*this->Scan, minPt, maxPt);
  for (auto k : KeypointTypes)
  {
    if (!this->Enabled[k])
      continue;
    this->Keypoints[k].Clear();
    this->Keypoints[k].Init(minPt.getVector3fMap(), maxPt.getVector3fMap());
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidateNotUsablePoints()
{
  // Loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      for (int index = 0; index < Npts; ++index)
        this->IsPointValid[scanLine][index] = false;
      continue;
    }

    // Invalidate first and last points: because of undistortion, the depth of
    // first and last points can not be compared
    for (int index = 0; index < this->NeighborWidth; ++index)
    {
      this->IsPointValid[scanLine][index] = false;
      this->IsPointValid[scanLine][Npts - 1 - index] = false;
    }

    // Loop over remaining points of the scan line
    for (int index = this->NeighborWidth; index < Npts - this->NeighborWidth; ++index)
    {
      const auto& currentPoint = scanLineCloud[index].getVector3fMap();
      // Invalidate points which are too close from the sensor
      if (currentPoint.norm() < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index] = false;
        continue;
      }

      // Invalidate occluded points due to depth gap.
      // A depth gap can be detected when the vectors between close neighbors and their beam directions are collinear
      // The threshold on the colinearity can exclude points on too bended walls
      // The farthest points must be invalidated because they might belong to an occluded area.
      const auto& prevPoint = scanLineCloud[index - 1].getVector3fMap();
      Eigen::Vector3f beamDir = prevPoint / prevPoint.norm();
      float azimuth = std::acos(std::abs(beamDir.dot(currentPoint) / currentPoint.norm()));
      float dotProduct = beamDir.dot(currentPoint - prevPoint);
      if (azimuth < 3 * this->AzimuthalResolution && std::abs(dotProduct) >= this->EdgeDepthGapThreshold)
      {
        if (dotProduct > 0)
          this->IsPointValid[scanLine][index] = false;
        else
          this->IsPointValid[scanLine][index - 1] = false;
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (!this->IsPointValid[scanLine][index])
        continue;

      // Fill left and right neighborhoods, from central point to sides.
      // /!\ The way the neighbors are added to the vectors matters,
      std::vector<int> leftNeighbors(this->NeighborWidth);
      std::vector<int> rightNeighbors(this->NeighborWidth);
      for (int j = index - 1; j >= index - this->NeighborWidth; --j)
        leftNeighbors[index - 1 - j] = j;
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
        rightNeighbors[j - index - 1] = j;

      // Compute intensity gap
      float gap = scanLineCloud[index].intensity - scanLineCloud[index - 1].intensity;
      float depthGap = (scanLineCloud[index - 1].getVector3fMap() - scanLineCloud[index].getVector3fMap()).norm();
      if (gap > this->EdgeIntensityGapThreshold && depthGap < 0.1)
      {
        // Compute mean intensity on the left
        float meanIntensityLeft = 0;
        for (int indexLeft : leftNeighbors)
          meanIntensityLeft += scanLineCloud[indexLeft].intensity;
        meanIntensityLeft /= this->NeighborWidth;
        // Compute mean intensity on the right
        float meanIntensityRight = 0;
        for (int indexRight : rightNeighbors)
          meanIntensityRight += scanLineCloud[indexRight].intensity;
        meanIntensityRight /= this->NeighborWidth;
        this->IntensityGap[scanLine][index] = std::abs(meanIntensityLeft - meanIntensityRight);
      }

      // central point
      const Point& currentPoint = scanLineCloud[index];
      const Eigen::Vector3f centralPoint = currentPoint.getVector3fMap();

      // We will compute the line that fits the neighbors located before the current point.
      // We will do the same for the neighbors located after the current point.
      // We will then compute the angle between these two lines as an approximation
      // of the "sharpness" of the current point.
      LineFitting leftLine, rightLine;

      // Fit line on the left and right neighborhoods and
      // Indicate if they are flat or not
      bool leftFlat = leftLine.FitPCAAndCheckConsistency(scanLineCloud, leftNeighbors);
      bool rightFlat = rightLine.FitPCAAndCheckConsistency(scanLineCloud, rightNeighbors);

      const float beamLineAngleLeft = std::acos(std::abs(leftLine.Direction.dot(centralPoint) / (centralPoint.norm())));
      const float beamLineAngleRight = std::acos(std::abs(rightLine.Direction.dot(centralPoint) / (centralPoint.norm())));

      leftFlat = leftFlat && beamLineAngleLeft > Utils::Deg2Rad(this->MinBeamSurfaceAngle);
      rightFlat = rightFlat && beamLineAngleRight > Utils::Deg2Rad(this->MinBeamSurfaceAngle);

      const auto& rightPt = scanLineCloud[index + 1].getVector3fMap();
      const auto& leftPt = scanLineCloud[index - 1].getVector3fMap();

      const float angleRight = std::acos(std::abs(rightPt.dot(centralPoint) / (rightPt.norm() * centralPoint.norm())));
      const float angleLeft = std::acos(std::abs(leftPt.dot(centralPoint) / (leftPt.norm() * centralPoint.norm())));

      if (leftFlat && rightFlat)
      {
        // Compute depth gap
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as an angle point if it is due to gap
        distLeft =  leftLine.DistanceToPoint(centralPoint);
        distRight = rightLine.DistanceToPoint(centralPoint);
        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);

        // Compute angles
        if ((distLeft < this->DistToLineThreshold) && (distRight < this->DistToLineThreshold))
        {
          this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
          // Remove previous point from angle inspection if the angle is not maximal locally
          if (this->Angles[scanLine][index] > this->EdgeSinAngleThreshold)
          {
            // Check previously computed angle to keep only the maximum angle keypoint locally
            for (int indexLeft : leftNeighbors)
            {
              if (this->Angles[scanLine][indexLeft] <= this->Angles[scanLine][index])
                this->Angles[scanLine][indexLeft] = -1;
              else
                this->Angles[scanLine][index] = -1;
            }
          }
        }
      }
      else if (!leftFlat && rightFlat)
      {
        // Compute depth gap
        distRight = rightLine.DistanceToPoint(centralPoint);

        distLeft = leftPt.dot(centralPoint - leftPt) / leftPt.norm();
        // Check points are consecutive + not on a bended wall
        // If the points lay on a bended wall, next points space gap should be greater
        float prevDistLeft = (leftPt - scanLineCloud[index - 2].getVector3fMap()).norm();
        if (angleLeft < 2 * this->AzimuthalResolution && (distLeft < prevDistLeft || distLeft < distRight))
          distLeft = -1.f;

        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
      }
      else if (leftFlat && !rightFlat)
      {
        // Compute depth gap
        distLeft = leftLine.DistanceToPoint(centralPoint);

        distRight = centralPoint.dot(rightPt - centralPoint) / centralPoint.norm();
        // Check points are consecutive + not on a bended wall
        // If the points lay on a bended wall, next points depth gap should be greater
        auto& nextRightPt = scanLineCloud[index + 2].getVector3fMap();
        float nextDistRight = rightPt.dot(nextRightPt - rightPt) / rightPt.norm();
        if (angleRight < 2 * this->AzimuthalResolution && (distRight < nextDistRight || distRight < distLeft))
          distRight = -1.f;

        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
      }
      else
      {
        distRight = centralPoint.dot(rightPt - centralPoint) / centralPoint.norm();
        // Check points are consecutive + not on a bended wall
        // If the points lay on a bended wall, next points depth gap should be greater
        auto& nextRightPt = scanLineCloud[index + 2].getVector3fMap();
        float nextDistRight = rightPt.dot(nextRightPt - rightPt) / rightPt.norm();
        if (angleRight < 2 * this->AzimuthalResolution && (distRight < nextDistRight || distRight < distLeft))
          distRight = -1.f;

        distLeft = leftPt.dot(centralPoint - leftPt) / leftPt.norm();
        // Check points are consecutive + not on a bended wall
        // If the points lay on a bended wall, next points space gap should be greater
        float prevDistLeft = (leftPt - scanLineCloud[index - 2].getVector3fMap()).norm();
        if (angleLeft < 2 * this->AzimuthalResolution && (distLeft < prevDistLeft || distLeft < distRight))
          distLeft = -1.f;

        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::AddKptsUsingCriterion (Keypoint k,
                                                             const std::vector<std::vector<float>>& values,
                                                             float threshold,
                                                             bool threshIsMax,
                                                             double weightBasis)
{
  // Loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanlineIdx = 0; scanlineIdx < static_cast<int>(this->NbLaserRings); ++scanlineIdx)
  {
    const int Npts = this->ScanLines[scanlineIdx]->size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // If threshIsMax : ascending order (lowest first)
    // If threshIsMin : descending order (greatest first)
    std::vector<size_t> sortedValuesIndices = Utils::SortIdx(values[scanlineIdx], threshIsMax);

    for (const auto& index: sortedValuesIndices)
    {
      // If the point is invalid, continue
      if (!this->IsPointValid[scanlineIdx][index] || values[scanlineIdx][index] < 0)
        continue;

      // Check criterion threshold
      bool valueAboveThresh = values[scanlineIdx][index] > threshold;
      // If criterion is not respected, break loop as indices are sorted.
      if ((threshIsMax && valueAboveThresh) ||
          (!threshIsMax && !valueAboveThresh))
        break;
      if (this->Label[scanlineIdx][index][Keypoint::EDGE])
        continue;

      float weight = threshIsMax? weightBasis + values[scanlineIdx][index] / threshold : weightBasis - values[scanlineIdx][index] / values[scanlineIdx][0];

      #pragma omp critical
      {
        // Indicate the type of the keypoint to debug and to exclude double edges
        this->Label[scanlineIdx][index].set(k);
        // Add keypoint
        this->Keypoints[k].AddPoint(this->ScanLines[scanlineIdx]->at(index), weight);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputePlanes()
{
  this->AddKptsUsingCriterion(Keypoint::PLANE, this->Angles, this->PlaneSinAngleThreshold);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeEdges()
{
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->Angles, this->EdgeSinAngleThreshold, false, 2);
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->DepthGap, this->EdgeDepthGapThreshold, false, 3);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  this->AddKptsUsingCriterion(Keypoint::INTENSITY_EDGE, this->IntensityGap, this->EdgeIntensityGapThreshold, false);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeBlobs()
{
  for (unsigned int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    for (unsigned int index = 0; index < this->ScanLines[scanLine]->size(); ++index)
    {
      if (this->IsPointValid[scanLine][index])
        this->Keypoints[Keypoint::BLOB].AddPoint(this->ScanLines[scanLine]->at(index));
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  for (const PointCloud::Ptr& scanLine : this->ScanLines)
  {
    for (unsigned int index = 1; index < scanLine->size(); ++index)
    {
      // Compute horizontal angle between two measurements
      // WARNING: to be correct, the points need to be in the LIDAR sensor
      // coordinates system, where the sensor is spinning around Z axis.
      Eigen::Map<const Eigen::Vector2f> p1(scanLine->at(index - 1).data);
      Eigen::Map<const Eigen::Vector2f> p2(scanLine->at(index).data);
      float angle = std::abs(std::acos(p1.dot(p2) / (p1.norm() * p2.norm())));

      // Keep only angles greater than 0 to avoid dual return issues
      if (angle > 1e-4)
        angles.push_back(angle);
    }
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
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["intensity_edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::INTENSITY_EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["valid"]          = get1DVector(this->IsPointValid);
  return map;
}

} // end of LidarSlam namespace