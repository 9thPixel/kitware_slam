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

namespace Utils
{
namespace
{
//-----------------------------------------------------------------------------
using KeypointFlags = std::underlying_type<Keypoint>::type;
// Check if kp is of a specific keypoint type
inline bool Check(KeypointFlags kp, Keypoint type) { return kp & (1 << type); }
// Set kp to be of specific type
inline void Set(KeypointFlags& kp, Keypoint type) { kp |= (1 << type); }
// Set kp to NOT be of specific type
inline void Unset(KeypointFlags& kp, Keypoint type) { kp &= ~(1 << type); }

//-----------------------------------------------------------------------------
// Positive k % n modulo operation
inline int Mod(int k, int n)
{
  return ((k %= n) < 0) ? k + n : k;
}

//-----------------------------------------------------------------------------
struct LineFitting
{
  //! Fitting using PCA
  bool FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
              const std::vector<int>& indices);

  //! Fitting using very local line and check if this local line is consistent
  //! in a more global neighborhood
  bool FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                 const std::vector<int>& indices);

  //! Compute the squared distance of a point to the fitted line
  inline float SquaredDistanceToPoint(Eigen::Vector3f const& point) const;

  // Direction and position
  Eigen::Vector3f Direction;
  Eigen::Vector3f Position;

  //! Max distance allowed from the farest point to estimated line to be considered as real line
  float MaxDistance = 0.02;  // [m]

  //! Max angle allowed between consecutive segments in the neighborhood to be considered as line
  float MaxAngle = DEG2RAD(40.);  // [rad]
};

//-----------------------------------------------------------------------------
bool LineFitting::FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                         const std::vector<int>& indices)
{
  // Compute PCA to determine best line approximation of the points distribution
  // and save points centroid in Position
  Eigen::Vector3f eigVals;
  Eigen::Matrix3f eigVecs;
  Utils::ComputeMeanAndPCA(cloud, indices, this->Position, eigVecs, eigVals);

  // Get Direction as main eigen vector
  this->Direction = eigVecs.col(2);

  // If a point of the neighborhood is too far from the fitted line,
  // we consider the neighborhood as non flat
  bool isLineFittingAccurate = true;
  const float sqMaxDistance = this->MaxDistance * this->MaxDistance;
  for (const auto& pointId: indices)
  {
    if (this->SquaredDistanceToPoint(cloud[pointId].getVector3fMap()) > sqMaxDistance)
    {
      isLineFittingAccurate = false;
      break;
    }
  }
  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  const float maxSinAngle = std::sin(this->MaxAngle);
  bool isLineFittingAccurate = true;

  // First check if the neighborhood is approximately straight
  const Eigen::Vector3f U = (cloud[indices.back()].getVector3fMap() - cloud[indices.front()].getVector3fMap()).normalized();
  for (unsigned int i = 0; i < indices.size() - 1; i++)
  {
    const Eigen::Vector3f V = (cloud[indices[i + 1]].getVector3fMap() - cloud[indices[i]].getVector3fMap()).normalized();
    const float sinAngle = (U.cross(V)).norm();
    if (sinAngle > maxSinAngle)
    {
      isLineFittingAccurate = false;
      break;
    }
  }

  // Then fit with PCA (only if isLineFittingAccurate is true)
  return isLineFittingAccurate && this->FitPCA(cloud, indices);
}

//-----------------------------------------------------------------------------
inline float LineFitting::SquaredDistanceToPoint(Eigen::Vector3f const& point) const
{
  return ((point - this->Position).cross(this->Direction)).squaredNorm();
}
} // end of anonymous namespace
} // end of Utils namespace

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Initialize the features vectors and keypoints clouds,
  // and project input points on vertex map
  this->PrepareDataForNewFrame();

  // Invalidate points with bad criteria
  this->InvalidateNotUsablePoints();

  // Compute keypoints scores
  this->ComputeFeatures();

  // Labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNewFrame()
{
  // Do not use clear(), otherwise weird things could happen if outer program
  // uses these pointers
  this->EdgesPoints.reset(new PointCloud);
  this->PlanarsPoints.reset(new PointCloud);
  this->BlobsPoints.reset(new PointCloud);
  Utils::CopyPointCloudMetadata(*this->Scan, *this->EdgesPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *this->PlanarsPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *this->BlobsPoints);

  // Estimate azimuthal resolution if not already done
  if (this->NbFiringsPerLaserRing <= 0)
    this->EstimateLaserRingsParameters();
  // Check the number of laser rings
  for (const Point& pt : *this->Scan)
  {
    if (pt.laser_id >= this->NbLaserRings)
      this->NbLaserRings = pt.laser_id;
  }

  // Initialize the features vectors with the correct size and default value
  this->Angles      .setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, 0.);
  this->Saliency    .setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, 0.);
  this->DepthGap    .setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, 0.);
  this->IntensityGap.setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, 0.);
  this->IsPointValid.setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, ~0);  // set all flags to 1
  this->Label       .setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing,  0);  // reset all flags to 0
  this->ScanIds     .setConstant(this->NbLaserRings, this->NbFiringsPerLaserRing, -1);

  // Fill vertex map indices
  const float azimuthalResolution = 2 * M_PI / this->NbFiringsPerLaserRing + 1e-6;
  #pragma omp parallel for num_threads(this->NbThreads) firstprivate(azimuthalResolution)
  for (int i = 0; i < static_cast<int>(this->Scan->size()); ++i)
  {
    const Point& pt = this->Scan->at(i);
    // Clockwise azimuth angle of each point, in range [0; 2pi[
    float azimuth = M_PI - std::atan2(pt.y, pt.x);
    // Store index of projected point
    int idxCol = azimuth / azimuthalResolution;
    this->ScanIds(pt.laser_id, idxCol) = i;
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidateNotUsablePoints()
{
  // Max angle between Lidar Ray and hyptothetic plane normal
  constexpr float MAX_ANGLE_TO_NORMAL = Utils::Deg2Rad(70.);
  // Coeff to multiply to point depth, in order to obtain the maximal distance
  // between two neighbors of the same Lidar ray on a plane
  const float azimuthalResolution = 2 * M_PI / this->NbFiringsPerLaserRing;
  const float maxPosDiffCoeff = std::sin(azimuthalResolution) / std::cos(azimuthalResolution + MAX_ANGLE_TO_NORMAL);

  // Loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) firstprivate(maxPosDiffCoeff)
  for (int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    // Timestamp and index of the previous valid point
    float lastValidPointTime = std::numeric_limits<float>::lowest();
    int lastValidPointIdx = 0;

    for (int index = 0; index < this->NbFiringsPerLaserRing; ++index)
    {
      // Check if we have a valid measurement, and get point
      const int ptIndex = this->ScanIds(scanLine, index);
      if (ptIndex < 0)
      {
        this->IsPointValid(scanLine, index) = 0;
        continue;
      }
      const Point& currentLidarPoint = this->Scan->at(ptIndex);
      const auto& currentPoint = currentLidarPoint.getVector3fMap();
      const float L = currentPoint.norm();

      // If a time gap is detected, this means that we have reached the end of
      // the scanline. Because of rolling shutter distortion, the neighborhoods
      // made by the points around this direction are invalid.
      if (currentLidarPoint.time < lastValidPointTime)
      {
        int timeGapIdx = (index + lastValidPointIdx) / 2;
        for (int i = -this->NeighborWidth; i <= this->NeighborWidth; ++i)
          this->IsPointValid(scanLine, Utils::Mod(timeGapIdx + i, this->NbFiringsPerLaserRing)) = 0;
      }
      lastValidPointIdx = index;
      lastValidPointTime = currentLidarPoint.time;

      // Invalidate points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid(scanLine, index) = 0;
      }

      // Extract left and right neighborhoods from vertex map.
      // If the neighborhood is empty: skip the point, next part will
      // use it as a depth gap boundary point.
      // If at least one neighbor is missing: skip and invalidate the point.
      std::vector<int> leftNeighbors = this->GetNeighbors(scanLine, index, -this->NeighborWidth);
      std::vector<int> rightNeighbors = this->GetNeighbors(scanLine, index, this->NeighborWidth);
      if (leftNeighbors.empty() || rightNeighbors.empty())
        continue;
      if (leftNeighbors.size() < static_cast<unsigned int>(this->NeighborWidth) ||
          rightNeighbors.size() < static_cast<unsigned int>(this->NeighborWidth))
      {
        this->IsPointValid(scanLine, index) = 0;
        continue;
      }

      // Compute maximal acceptable distance of two consecutive neighbors
      // aquired by the same laser, considering they lay on the same plane which
      // is not too oblique relatively to Lidar ray.
      // Check that this expected distance is not below range measurements noise.
      const float maxPosDiff = std::max(L * maxPosDiffCoeff, 0.02f);
      const float sqMaxPosDiff = maxPosDiff * maxPosDiff;

      // Invalidate occluded points due to depth gap or parallel beam.
      // If the distance between two successive points is bigger than the
      // expected length, it means that there is a depth gap. In this case, we
      // must invalidate the farthest points which belong to the occluded area.
      const auto& nextPoint = this->Scan->at(rightNeighbors[0]).getVector3fMap();
      if ((nextPoint - currentPoint).squaredNorm() > sqMaxPosDiff)
      {
        // If current point is the closest, next part is invalidated, starting from next point
        if (L < nextPoint.norm())
        {
          this->IsPointValid(scanLine, index + 1) = 0;
          for (unsigned int i = 1; i < rightNeighbors.size(); ++i)
          {
            const auto& Y  = this->Scan->at(rightNeighbors[i - 1]).getVector3fMap();
            const auto& Yn = this->Scan->at(rightNeighbors[i]).getVector3fMap();
            // If there is a new gap in the neighborhood, 
            // the remaining points of the neighborhood are kept.
            if ((Yn - Y).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the current neighbor point is disabled
            // NOTE: this is only valid as there is no missing points in the neighborhood
            this->IsPointValid(scanLine, Utils::Mod(index + i + 1, this->NbFiringsPerLaserRing)) = 0;
          }
        }
        // If current point is the farthest, invalidate previous part, starting from current point
        else
        {
          this->IsPointValid(scanLine, index) = 0;
          for (unsigned int i = 1; i < leftNeighbors.size(); ++i)
          {
            const auto& Y  = this->Scan->at(leftNeighbors[i - 1]).getVector3fMap();
            const auto& Yp = this->Scan->at(leftNeighbors[i]).getVector3fMap();
            // If there is a new gap in the neighborhood, 
            // the remaining points of the neighborhood are kept.
            if ((Y - Yp).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the previous neighbor point is disabled
            // NOTE: this is only valid as there is no missing points in the neighborhood
            this->IsPointValid(scanLine, Utils::Mod(index - i - 1, this->NbFiringsPerLaserRing)) = 0;
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeFeatures()
{
  const float sqDistToLineThreshold = this->DistToLineThreshold * this->DistToLineThreshold;  // [m²]
  const float sqDepthDistCoeff = 0.25;
  const float minDepthGapDist = 1.5;  // [m]

  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(sqDistToLineThreshold, sqDepthDistCoeff, minDepthGapDist)
  for (int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    // Loop over points in the current scan line
    for (int index = 0; index < this->NbFiringsPerLaserRing; ++index)
    {
      // Skip curvature computation for invalid points
      if (!this->IsPointValid(scanLine, index))
        continue;

      // Central point
      const Point& currentPoint = this->Scan->at(this->ScanIds(scanLine, index));
      const auto& centralPoint = currentPoint.getVector3fMap();

      // Extract left and right neighborhoods from vertex map.
      std::vector<int> leftNeighbors = this->GetNeighbors(scanLine, index, -this->NeighborWidth);
      std::vector<int> rightNeighbors = this->GetNeighbors(scanLine, index, this->NeighborWidth);

      // If all neighbors are missing, keep the point as depth gap boundary
      if (leftNeighbors.empty() || rightNeighbors.empty())
      {
        this->DepthGap(scanLine, index) = std::numeric_limits<float>::max();
        continue;
      }

      // Compute intensity gap
      this->IntensityGap(scanLine, index) = std::abs(this->Scan->at(rightNeighbors[0]).intensity
                                                    - this->Scan->at(leftNeighbors[0]).intensity);

      // Fit line on the left and right neighborhoods and
      // Indicate if they are flat or not
      Utils::LineFitting leftLine, rightLine;
      const bool leftFlat = leftLine.FitPCAAndCheckConsistency(*this->Scan, leftNeighbors);
      const bool rightFlat = rightLine.FitPCAAndCheckConsistency(*this->Scan, rightNeighbors);

      // Measurement of the depth gap
      float distLeft = 0., distRight = 0.;

      // If both neighborhoods are flat, we can compute the angle between them
      // as an approximation of the sharpness of the current point
      if (leftFlat && rightFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as an angle point if it is due to gap
        distLeft = leftLine.SquaredDistanceToPoint(centralPoint);
        distRight = rightLine.SquaredDistanceToPoint(centralPoint);

        // If current point is not too far from estimated lines,
        // save the sin of angle between these two lines
        if ((distLeft < sqDistToLineThreshold) && (distRight < sqDistToLineThreshold))
          this->Angles(scanLine, index) = (leftLine.Direction.cross(rightLine.Direction)).norm();
      }

      // Here one side of the neighborhood is non flat.
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      // CHECK : looks strange to estimate depth gap without considering current point
      else if (!leftFlat && rightFlat)
      {
        distLeft = std::numeric_limits<float>::max();
        for (const auto& leftNeighborId: leftNeighbors)
        {
          const auto& leftNeighbor = this->Scan->at(leftNeighborId).getVector3fMap();
          distLeft = std::min(distLeft, rightLine.SquaredDistanceToPoint(leftNeighbor));
        }
        distLeft *= sqDepthDistCoeff;
      }
      else if (leftFlat && !rightFlat)
      {
        distRight = std::numeric_limits<float>::max();
        for (const auto& rightNeighborId: rightNeighbors)
        {
          const auto& rightNeighbor = this->Scan->at(rightNeighborId).getVector3fMap();
          distRight = std::min(distRight, leftLine.SquaredDistanceToPoint(rightNeighbor));
        }
        distRight *= sqDepthDistCoeff;
      }

      // No neighborhood is flat.
      // We will compute saliency of the current keypoint from its far neighbors.
      else
      {
        // Compute salient point score
        const float sqCurrDepth = centralPoint.squaredNorm();
        bool hasLeftEncounteredDepthGap = false;
        bool hasRightEncounteredDepthGap = false;

        std::vector<int> farNeighbors;
        farNeighbors.reserve(2 * this->NeighborWidth);

        // The salient point score is the distance between the current point
        // and the points that have a depth gap with the current point
        // CHECK : consider only consecutive far neighbors, starting from the central point.
        for (const auto& leftNeighborId: leftNeighbors)
        {
          // Left neighborhood depth gap computation
          if (std::abs(this->Scan->at(leftNeighborId).getVector3fMap().squaredNorm() - sqCurrDepth) > minDepthGapDist)
          {
            hasLeftEncounteredDepthGap = true;
            farNeighbors.emplace_back(leftNeighborId);
          }
          else if (hasLeftEncounteredDepthGap)
            break;
        }
        for (const auto& rightNeighborId: rightNeighbors)
        {
          // Right neigborhood depth gap computation
          if (std::abs(this->Scan->at(rightNeighborId).getVector3fMap().squaredNorm() - sqCurrDepth) > minDepthGapDist)
          {
            hasRightEncounteredDepthGap = true;
            farNeighbors.emplace_back(rightNeighborId);
          }
          else if (hasRightEncounteredDepthGap)
            break;
        }

        // If there are enough neighbors with a big depth gap,
        // we propose to compute the saliency of the current point
        // as the distance between the line that roughly fits the far neighbors
        // with a depth gap and the current point
        if (farNeighbors.size() > static_cast<unsigned int>(this->NeighborWidth))
        {
          Utils::LineFitting farNeighborsLine;
          farNeighborsLine.FitPCA(*this->Scan, farNeighbors);
          this->Saliency(scanLine, index) = farNeighborsLine.SquaredDistanceToPoint(centralPoint);
        }
      }

      // Store max depth gap
      this->DepthGap(scanLine, index) = std::max(distLeft, distRight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  const float sqEdgeSaliencythreshold = this->EdgeSaliencyThreshold * this->EdgeSaliencyThreshold;
  const float sqEdgeDepthGapThreshold = this->EdgeDepthGapThreshold * this->EdgeDepthGapThreshold;

  // loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(sqEdgeSaliencythreshold, sqEdgeDepthGapThreshold)
  for (int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    // Sort the curvature score in a decreasing order
    std::vector<size_t> sortedDepthGapIdx  = Utils::SortIdx(this->DepthGap    .row(scanLine), false);
    std::vector<size_t> sortedAnglesIdx    = Utils::SortIdx(this->Angles      .row(scanLine), false);
    std::vector<size_t> sortedSaliencyIdx  = Utils::SortIdx(this->Saliency    .row(scanLine), false);
    std::vector<size_t> sortedIntensityGap = Utils::SortIdx(this->IntensityGap.row(scanLine), false);

    // Add edge according to criterion
    auto addEdgesUsingCriterion = [this, scanLine](const std::vector<size_t>& sortedValuesIdx,
                                                   const VertexMap<float>& values,
                                                   float threshold,
                                                   int invalidNeighborhoodSize)
    {
      for (int index: sortedValuesIdx)
      {
        // Check criterion threshold
        // If criterion is not respected, break loop as indices are sorted in decreasing order.
        if (values(scanLine, index) < threshold)
          break;

        // If the point is invalid as edge, continue
        if (!Utils::Check(this->IsPointValid(scanLine, index), Keypoint::EDGE))
          continue;

        // Else indicate that the point is an edge
        Utils::Set(this->Label(scanLine, index), Keypoint::EDGE);

        // Invalid its neighbors
        for (int j = index - invalidNeighborhoodSize; j <= index + invalidNeighborhoodSize; ++j)
          Utils::Unset(this->IsPointValid(scanLine, Utils::Mod(j, this->NbFiringsPerLaserRing)), Keypoint::EDGE);
        Utils::Set(this->IsPointValid(scanLine, index), Keypoint::EDGE);
      }
    };

    // Edges using depth gap
    addEdgesUsingCriterion(sortedDepthGapIdx, this->DepthGap, sqEdgeDepthGapThreshold, this->NeighborWidth - 1);
    // Edges using angles
    addEdgesUsingCriterion(sortedAnglesIdx, this->Angles, this->EdgeSinAngleThreshold, this->NeighborWidth);
    // Edges using saliency
    addEdgesUsingCriterion(sortedSaliencyIdx, this->Saliency, sqEdgeSaliencythreshold, this->NeighborWidth - 1);
    // Edges using intensity
    addEdgesUsingCriterion(sortedIntensityGap, this->IntensityGap, this->EdgeIntensityGapThreshold, 1);

    // Planes (using angles)
    for (int k = this->NbFiringsPerLaserRing - 1; k >= 0; --k)
    {
      int index = sortedAnglesIdx[k];
      const float sinAngle = this->Angles(scanLine, index);

      // thresh
      if (sinAngle > this->PlaneSinAngleThreshold)
        break;

      // if the point is invalid as plane or sinAngle value is unset, continue
      if (!Utils::Check(this->IsPointValid(scanLine, index), Keypoint::PLANE) || sinAngle < 1e-6)
        continue;

      // else indicate that the point is a planar one
      Utils::Set(this->Label(scanLine, index), Keypoint::PLANE);

      // Invalid its neighbors so that we don't have too
      // many planar keypoints in the same region. This is
      // required because of the k-nearest search + plane
      // approximation realized in the odometry part. Indeed,
      // if all the planar points are on the same scan line the
      // problem is degenerated since all the points are distributed
      // on a line.
      for (int j = index - 4; j <= index + 4; ++j)
        Utils::Unset(this->IsPointValid(scanLine, Utils::Mod(j, this->NbFiringsPerLaserRing)), Keypoint::PLANE);
      Utils::Set(this->IsPointValid(scanLine, index), Keypoint::PLANE);
    }

    // Blobs Points
    // CHECK : why using only 1 point over 3?
    // TODO : disable blobs if not required
    for (int index = 0; index < this->NbFiringsPerLaserRing; index += 3)
    {
      if (Utils::Check(this->IsPointValid(scanLine, index), Keypoint::BLOB))
        Utils::Set(this->Label(scanLine, index), Keypoint::BLOB);
    }
  }

  auto addKeypoints = [this](Keypoint type, PointCloud::Ptr& keypoints)
  {
    for (unsigned int i = 0; i < this->ScanIds.size(); i++)
    {
      if (Utils::Check(this->Label(i), type))
        keypoints->push_back(this->Scan->at(this->ScanIds(i)));
    }
  };
  addKeypoints(Keypoint::EDGE, this->EdgesPoints);
  addKeypoints(Keypoint::PLANE, this->PlanarsPoints);
  addKeypoints(Keypoint::BLOB, this->BlobsPoints);
}

//-----------------------------------------------------------------------------
std::vector<int> SpinningSensorKeypointExtractor::GetNeighbors(int laserRing, int firingIdx, int nbNeighborsDist)
{
  std::vector<int> neighbors;
  const int direction = (nbNeighborsDist > 0) ? 1 : -1;
  for (int firingIdxOffset = 1; firingIdxOffset <= std::abs(nbNeighborsDist); ++firingIdxOffset)
  {
    const int ptIndex = this->ScanIds(laserRing, Utils::Mod(firingIdx + direction * firingIdxOffset, this->NbFiringsPerLaserRing));
    if (ptIndex >= 0)
      neighbors.push_back(ptIndex);
  }
  return neighbors;
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::EstimateLaserRingsParameters()
{
  // Separate pointcloud into different scan lines
  std::vector<PointCloud::Ptr> scanLines;
  for (const Point& point : *this->Scan)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= scanLines.size())
      scanLines.emplace_back(new PointCloud);

    // Copy the current point to its corresponding laser scan
    scanLines[point.laser_id]->push_back(point);
  }

  // Compute horizontal angle values between successive points
  // WARNING: This makes the assumption that the points in each scan line are
  // already stored in sorted order, by increasing azimuth/timestamp.
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  for (const PointCloud::Ptr& scanLine : scanLines)
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

  // Save the computed parameters
  this->NbLaserRings = scanLines.size();
  this->NbFiringsPerLaserRing = std::ceil(2 * M_PI / medianAngle);
  std::cout << "Estimated spinning LiDAR sensor mode : "
            << this->NbLaserRings << " x " << this->NbFiringsPerLaserRing
            << " (" << Utils::Rad2Deg(medianAngle) << "°)" << std::endl;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vertexMap)
  {
    std::vector<float> vec(this->Scan->size());
    for (unsigned int i = 0; i < this->ScanIds.size(); i++)
    {
      const int ptIdx = this->ScanIds(i);
      if (ptIdx >= 0)
        vec[ptIdx] = vertexMap(i);
    }
    return vec;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vertexMap, Keypoint flag)
  {
    std::vector<float> vec(this->Scan->size());
    for (unsigned int i = 0; i < this->ScanIds.size(); i++)
    {
      const int ptIdx = this->ScanIds(i);
      if (ptIdx >= 0)
        vec[ptIdx] = Utils::Check(vertexMap(i), flag);
    }
    return vec;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["saliency"]       = get1DVector(this->Saliency);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["edge_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::EDGE);
  map["plane_validity"] = get1DVectorFromFlag(this->IsPointValid, Keypoint::PLANE);
  map["blob_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::BLOB);
  return map;
}

} // end of LidarSlam namespace