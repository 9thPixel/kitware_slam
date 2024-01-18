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
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypointsCloud(new PointCloud);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypointsCloud);

  std::vector<LidarPoint> points = this->Keypoints.at(k);
  keypointsCloud->reserve(points.size());
  for (const auto& pt : points)
    keypointsCloud->push_back(pt);

  return keypointsCloud;
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
void DenseSpinningSensorKeypointExtractor::ClearKeypoints()
{
  for (const auto& k : KeypointTypes)
  {
    if (this->Keypoints.count(k))
      this->Keypoints[k].clear();
    if (this->Enabled[k])
      this->Keypoints[k].reserve(this->Scan->size());
  }
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
  OUTPUT_FEATURE("/tmp/SpaceGapH.pgm", SpaceGapH, 1.);
  OUTPUT_FEATURE("/tmp/DepthGapH.pgm", DepthGapH, 0.1);
  OUTPUT_FEATURE("/tmp/IntensityGapH.pgm", IntensityGapH, 1.);
  OUTPUT_FEATURE("/tmp/Angles.pgm", Angle, 1.);
}

//-----------------------------------------------------------------------------
bool DenseSpinningSensorKeypointExtractor::OutputKeypoints()
{
  for (const auto& type : this->Keypoints)
  {
    const Keypoint& k = type.first;
    int width = this->WidthVM;
    int height = this->HeightVM;

    std::ofstream fileCsv("/tmp/keypoints_" + KeypointTypeNames.at(k) + ".csv");
    fileCsv << "x,y,z,kpt\n";

    std::ofstream filePgm("/tmp/keypoints_" + KeypointTypeNames.at(k) + ".pgm");
    filePgm << "P2\n";
    filePgm << width << " " << height << "\n";
    filePgm << 1 << "\n";

    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        const auto& ptFeat = this->VertexMap[i][j];
        if (ptFeat == nullptr)
          continue;
        const Point& point = this->Scan->at(ptFeat->Index);
        int isKpt = ptFeat->KptTypes[k] ? 1 : 0;
        fileCsv << point.x << "," << point.y << "," << point.z << "," << isKpt << "\n";
        filePgm << isKpt << " ";
      }
      filePgm << "\n";
    }
    fileCsv.close();
    filePgm.close();
  }
  return true;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  this->InitInternalParameters();

  this->ClearKeypoints();

  this->CreateVertexMap();

  this->ComputeCurvature();

  // Label and extract keypoints
  //! Warning : order matters
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();

  this-> ClearVertexMap();
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Compute useful const values to lighten the loop
  float azimuthMinRad = Utils::Deg2Rad(this->AzimuthMin);
  float azimuthMaxRad = Utils::Deg2Rad(this->AzimuthMax);

  // Rescale angles in [0, 2pi]
  this->RescaleAngle(azimuthMinRad);
  this->RescaleAngle(azimuthMaxRad);

  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->HeightVM; ++i)
  {
    // If the line is almost empty, skip it
    int nPoints = this->GetScanLineSize(this->VertexMap[i]);
    if (this->IsScanLineAlmostEmpty(nPoints))
      continue;

    // Useful index to skip first and last points of the scan line
    int idxInLine = 0;

    for (unsigned int j = 0; j < this->WidthVM; ++j)
    {
      // PtFeat struct associated with current point
      const auto& currentFeat = this->VertexMap[i][j];

      // Ignore empty pixels
      if (!currentFeat)
        continue;

      // Count every valid point in the scan line, to be compared with nPoints later
      idxInLine++;

      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;

      // Central point
      const Point& currentPoint = this->Scan->at(currentFeat->Index);
      const Eigen::Vector3f& centralPoint = currentPoint.getVector3fMap();

      if (!this->CheckDistanceToSensor(currentFeat->Depth))
        continue;

      if (!this->CheckAzimuthAngle(azimuthMinRad, azimuthMaxRad, centralPoint))
        continue;

      // Fill neighbors (vectors of indices) for each side (left and right)
      // Those points must be more numerous than MinNeighNb and occupy more space than MinNeighRadius
      auto getNeighbors = [&](bool right, std::vector<int>& neighbors)
      {
        neighbors.reserve(this->WidthVM);
        neighbors.emplace_back(currentFeat->Index);
        int rightOrLeft = right ? 1 : -1;
        int idxNeigh = 1;
        float lineLength = 0.f;
        do {
          const auto& ptrFeat = this->VertexMap[i][(j + rightOrLeft * idxNeigh + this->WidthVM) % this->WidthVM];
          if (ptrFeat != nullptr)
          {
            neighbors.emplace_back(ptrFeat->Index);
            if (lineLength < MinNeighRadius)
              lineLength = (this->Scan->at(neighbors.back()).getVector3fMap() - this->Scan->at(neighbors.front()).getVector3fMap()).norm();
          }
          ++idxNeigh;
        }
        while ((lineLength < this->MinNeighRadius ||
               int(neighbors.size()) < this->MinNeighNb) &&
               int(neighbors.size()) < nPoints);
        neighbors.shrink_to_fit();
      };

      std::vector<int> leftNeighbors, rightNeighbors;
      getNeighbors(false, leftNeighbors);
      getNeighbors(true, rightNeighbors);

      // Stop search for first and last points of the scan line
      // because the discontinuity may alter the other criteria detection
      if (idxInLine < int(leftNeighbors.size()) || idxInLine >= nPoints - int(rightNeighbors.size()))
        continue;

      if (this->Enabled[EDGE])
      {
        // ---- Compute horizontal space gap ----

        // Find the first empty neighbor on the right and on the left
        auto& idxRightNeigh = this->Pc2VmIndices[rightNeighbors[1]];
        int nbEmptyRightNeigh = idxRightNeigh.Col - j;

        auto& idxLeftNeigh = this->Pc2VmIndices[leftNeighbors[1]];
        int nbEmptyLeftNeigh = j - idxLeftNeigh.Col;

        const float cosMinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle));

        float distRight = -1.f;
        float distLeft = -1.f;

        if (nbEmptyRightNeigh >= this->EdgeNbGapPoints)
        {
          const auto& rightPt = this->Scan->at(rightNeighbors[1]).getVector3fMap();
          float diffRightNorm = (rightPt - centralPoint).norm();
          float cosBeamLineAngleRight = std::abs((rightPt - centralPoint).dot(centralPoint) / (diffRightNorm * currentFeat->Depth));
          if (cosBeamLineAngleRight < cosMinBeamSurfaceAngle)
            distRight = diffRightNorm;
        }
        if (nbEmptyLeftNeigh >= this->EdgeNbGapPoints)
        {
          const auto& leftPt = this->Scan->at(leftNeighbors[1]).getVector3fMap();
          float diffLeftNorm = (leftPt - centralPoint).norm();
          float cosBeamLineAngleLeft = std::abs((leftPt - centralPoint).dot(centralPoint) / (diffLeftNorm * currentFeat->Depth));
          if (cosBeamLineAngleLeft < cosMinBeamSurfaceAngle)
            distLeft = diffLeftNorm;
        }
        currentFeat->SpaceGapH = std::max(distLeft, distRight);

        // Stop search for first and last points of the scan line
        // because the discontinuity may alter the other criteria detection
        if (idxInLine < int(leftNeighbors.size()) || idxInLine >= nPoints - int(rightNeighbors.size()))
          continue;

        // ---- Compute horizontal depth gap ----

        int idxNext = this->VertexMap[i][j + 1] ? j + 1 : this->VertexMap[i][j + 2] ? j + 2 : -1;
        int idxPrev = this->VertexMap[i][j - 1] ? j - 1 : this->VertexMap[i][j - 2] ? j - 2 : -1;
        if (idxNext > 0 && idxPrev > 0)
        {
          auto& directRightNeigh = this->VertexMap[i][idxNext];
          float distRight = directRightNeigh->Depth - currentFeat->Depth;
          auto& directLeftNeigh = this->VertexMap[i][idxPrev];
          float distLeft = directLeftNeigh->Depth - currentFeat->Depth;

          auto& postRightNeigh = this->VertexMap[i][idxNext + 1];
          auto& preLeftNeigh = this->VertexMap[i][idxPrev - 1];
          if (postRightNeigh != nullptr)
          {
            float distPostRight = postRightNeigh->Depth - directRightNeigh->Depth;
            if (distRight < distPostRight)
              distRight = 0.0f;
          }
          if (preLeftNeigh != nullptr)
          {
            float distPreLeft = preLeftNeigh->Depth - directLeftNeigh->Depth;
            if (distLeft < distPreLeft)
              distLeft = 0.0f;
          }
          currentFeat->DepthGapH = std::abs(distLeft) > std::abs(distRight) ? distLeft : distRight;
        }
      }

      if (currentFeat->SpaceGapH > this->EdgeDepthGapThreshold || currentFeat->DepthGapH > this->EdgeDepthGapThreshold)
        continue;

      // ---- Compute intensity gap ----

      LineFitting leftLine, rightLine;
      // Fit line on the left and right neighborhoods and
      // skip point if they are not usable
      if (!leftLine.FitLineAndCheckConsistency(*this->Scan, leftNeighbors) ||
          !rightLine.FitLineAndCheckConsistency(*this->Scan, rightNeighbors))
        continue;

      if (this->Enabled[INTENSITY_EDGE])
      {
        if (std::abs(this->Scan->at(rightNeighbors[1]).intensity - this->Scan->at(leftNeighbors[1]).intensity) > this->EdgeIntensityGapThreshold)
        {
          // Compute mean intensity on the left
          // We sample neighborhoods for computation time concerns
          float meanIntensityLeft = 0.f;
          int step = leftNeighbors.size() > this->MinNeighNb ? leftNeighbors.size() / this->MinNeighNb : 1;
          int cptMean = 0;
          // The first element of the neighborhood is the central point itself so we skip it
          for (int i = 1; i < leftNeighbors.size(); i += step)
          {
            meanIntensityLeft += this->Scan->at(leftNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityLeft /= cptMean;
          // Compute mean intensity on the right
          float meanIntensityRight = 0.f;
          step = rightNeighbors.size() > this->MinNeighNb ? rightNeighbors.size() / this->MinNeighNb : 1;
          cptMean = 0;
          for (int i = 1; i < rightNeighbors.size(); i += step)
          {
            meanIntensityRight += this->Scan->at(rightNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityRight /= cptMean;
          currentFeat->IntensityGapH = std::abs(meanIntensityLeft - meanIntensityRight);

          // Remove neighbor points to get the best intensity discontinuity locally
          auto neighPtr = this->GetPtFeat(leftNeighbors[1]);
          if (neighPtr->IntensityGapH < currentFeat->IntensityGapH)
            neighPtr->IntensityGapH = -1;
          else
            currentFeat->IntensityGapH = -1;
        }
      }

      // ---- Compute angle ----

      if (this->Enabled[PLANE] || this->Enabled[EDGE])
      {
        // Compute angles
        currentFeat->Angle = leftLine.Direction.dot(rightLine.Direction);
        // Remove angles too small to be edges
        if (currentFeat->Angle > -this->PlaneCosAngleThreshold)
        {
          currentFeat->Angle = 1.f;
          continue;
        }

        // Remove previous point from angle inspection if the angle is not maximal locally
        if (this->Enabled[EDGE] && currentFeat->Angle > this->EdgeCosAngleThreshold)
        {
          // Check previously computed angle to keep only the maximal angle keypoint locally
          for (int idx = 1; idx < leftNeighbors.size(); idx++)
          {
            std::shared_ptr<PtFeat> neighPtr = this->GetPtFeat(leftNeighbors[idx]);
            if (neighPtr->Angle > this->EdgeCosAngleThreshold &&
                neighPtr->Angle < -this->PlaneCosAngleThreshold)
            {
              if (neighPtr->Angle > currentFeat->Angle)
                currentFeat->Angle = 1.f;
              else
                neighPtr->Angle = 1.f;
              break;
            }
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKeypoint(const Keypoint& k, const LidarPoint& point)
{
  this->Keypoints.at(k).push_back(point);
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::Create2DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid)
{
  int nbPatchesX = std::ceil(this->WidthVM / this->PatchSize);
  int nbPatchesY = std::ceil(this->HeightVM / this->PatchSize);
  this->PatchGrid.reserve(nbPatchesX * nbPatchesY);

  // Fill grid with smart pointers to PtFeat
  for (unsigned int i = 0; i < this->HeightVM; ++i)
  {
    for (unsigned int j = 0; j < this->WidthVM; ++j)
    {
      const auto& ptFeat = this->VertexMap[i][j];
      if (!isPtFeatValid(ptFeat))
        continue;
      int idxPatchY = ((i + this->HeightVM) % this->HeightVM) / this->PatchSize;
      int idxPatchX = ((j + this->WidthVM)  % this->WidthVM)  / this->PatchSize;
      int indexPatch = idxPatchY * nbPatchesX + idxPatchX;
      this->PatchGrid[indexPatch].emplace_back(ptFeat);
      this->NbPointsInGrid++;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::Create3DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid)
{
  // Find width and height of the scan (in meters)
  Eigen::Vector4f ptMax, ptMin;
  pcl::getMinMax3D(*this->Scan, ptMax, ptMin);
  int xSize = std::abs(ptMax.x() - ptMin.x());
  int ySize = std::abs(ptMax.y() - ptMin.y());
  int zSize = std::abs(ptMax.z() - ptMin.z());

  // Compute the number of voxels in each direction
  int nbVoxelsX = std::ceil(xSize / this->VoxelDim);
  int nbVoxelsY = std::ceil(ySize / this->VoxelDim);
  int nbVoxelsZ = std::ceil(zSize / this->VoxelDim);
  // Reserve memory for the grid
  this->VoxGrid.reserve(nbVoxelsX * nbVoxelsY * nbVoxelsZ);

  // Compute the 3D position of the center of the first voxel
  // Considering that the voxel grid is centered on th
  Eigen::Array3f voxelGridOrigin = ptMin.head(3).array();

  // Fill grid with smart pointers to PtFeat
  for (unsigned int i = 0; i < this->Scan->size(); i++)
  {
    const LidarPoint& point = this->Scan->at(i);
    auto ptFeat = this->GetPtFeat(i);
    if (!isPtFeatValid(ptFeat))
      continue;
    Eigen::Array3i voxel = ((point.getArray3fMap() - voxelGridOrigin) / this->VoxelDim).array().round().template cast<int>();
    int index1D = voxel.z() * nbVoxelsX * nbVoxelsY + voxel.y() * nbVoxelsY + voxel.x();
    this->VoxGrid[index1D].push_back(ptFeat);
    this->NbPointsInGrid++;
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ClearGrid(std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>>& grid)
{
  grid.clear();
  this->NbPointsInGrid = 0;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKptsUsingGrid(Keypoint k,
                                                            std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>>& grid,
                                                            std::function<bool(const std::shared_ptr<PtFeat>&, const std::shared_ptr<PtFeat>&)> comparePtFeat)
{
  // If we have less candidates than the max keypoints number
  if (this->NbPointsInGrid < this->MaxPoints)
  {
    for (auto& cell : grid)
    {
      auto& vec = cell.second;
      for (auto& pt : vec)
      {
        this->AddKeypoint(k, this->Scan->at(pt->Index));
        pt->KptTypes.set(k);
      }
    }
    return;
  }

  // While nbkeypointextracted < nbpointmax && remains keypoints candidates
  int ptIdx = 0;
  while (ptIdx < this->MaxPoints)
  {
    bool remainKptCandidate = false;
    for (auto& cell : grid)
    {
      auto& vec = cell.second;
      // Check if cell has point that could be keypoints
      bool hasKptCandidate = std::any_of(vec.begin(), vec.end(), [&](const std::shared_ptr<PtFeat>& pt) {return ~pt->KptTypes[k];});
      if (!hasKptCandidate)
        continue;
      remainKptCandidate = true;

      // Search for max element in patch using comparePtFeat to compare all feats simultaneously
      auto maxPtIt = std::max_element(vec.begin(), vec.end(), comparePtFeat);
      const auto& maxPt = *maxPtIt;

      // Add to keypoints
      this->AddKeypoint(k, this->Scan->at(maxPt->Index));
      maxPt->KptTypes.set(k);
      ++ptIdx;

      if (ptIdx >= this->MaxPoints)
        break;
    }
    if (!remainKptCandidate)
      break;
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputePlanes()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &
           ~(pt->KptTypes[EDGE] | pt->KptTypes[PLANE]) &&
           pt->Angle < this->PlaneCosAngleThreshold;
  };
  switch (this->SamplingDSSKE[Keypoint::PLANE])
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->ClearGrid(this->PatchGrid);
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::PLANE,
                             this->PatchGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                              if (!isPtValid(a) && isPtValid(b))
                                return true;  // b is considered greater when a is nullptr
                              else if (isPtValid(a) && !isPtValid(b))
                                return false;   // a is considered greater when b is nullptr
                              else if (!isPtValid(a) && !isPtValid(b))
                                return true;  // Both are nullptr, no preference

                              return (a->Angle > b->Angle);
                             });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->ClearGrid(this->VoxGrid);
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::PLANE,
                             this->VoxGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               return (a->Angle > b->Angle);
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeEdges()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &&
           ~(pt->KptTypes[PLANE] | pt->KptTypes[INTENSITY_EDGE] | pt->KptTypes[EDGE]) &&
           ((pt->DepthGapH - (-1.0f) > 1e-6 && pt->DepthGapH > this->EdgeDepthGapThreshold) ||
            (pt->Angle < -this->PlaneCosAngleThreshold && pt->Angle > this->EdgeCosAngleThreshold) ||
            (pt->SpaceGapH - (-1.0f) > 1e-6 && pt->SpaceGapH > this->EdgeDepthGapThreshold));
  };
  switch (this->SamplingDSSKE[Keypoint::EDGE])
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->ClearGrid(this->PatchGrid);
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::EDGE,
                             this->PatchGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->DepthGapH < b->DepthGapH && b->DepthGapH > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->DepthGapH < a->DepthGapH && a->DepthGapH > this->EdgeDepthGapThreshold)
                                 return false;

                               if (a->Angle < b->Angle && b->Angle < -this->PlaneCosAngleThreshold && b->Angle > this->EdgeCosAngleThreshold)
                                 return true;
                               if (b->Angle < a->Angle && a->Angle < -this->PlaneCosAngleThreshold && a->Angle > this->EdgeCosAngleThreshold)
                                 return false;

                               if (a->SpaceGapH < b->SpaceGapH && b->SpaceGapH > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->SpaceGapH < a->SpaceGapH && a->SpaceGapH > this->EdgeDepthGapThreshold)
                                 return false;

                                return true;
                              });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->ClearGrid(this->VoxGrid);
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::EDGE,
                             this->VoxGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->DepthGapH < b->DepthGapH && b->DepthGapH > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->DepthGapH < a->DepthGapH && a->DepthGapH > this->EdgeDepthGapThreshold)
                                 return false;

                               if (a->Angle < b->Angle && b->Angle < -this->PlaneCosAngleThreshold && b->Angle > this->EdgeCosAngleThreshold)
                                 return true;
                               if (b->Angle < a->Angle && a->Angle < -this->PlaneCosAngleThreshold && a->Angle > this->EdgeCosAngleThreshold)
                                 return false;

                               if (a->SpaceGapH < b->SpaceGapH && b->SpaceGapH > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->SpaceGapH < a->SpaceGapH && a->SpaceGapH > this->EdgeDepthGapThreshold)
                                 return false;

                               return true;
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &&
           ~(pt->KptTypes[EDGE] | pt->KptTypes[INTENSITY_EDGE]) &&
           (pt->IntensityGapH - (-1.0f) > 1e-6 && pt->IntensityGapH > this->EdgeIntensityGapThreshold);
  };
  switch (this->SamplingDSSKE[Keypoint::INTENSITY_EDGE])
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->ClearGrid(this->PatchGrid);
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::INTENSITY_EDGE,
                             this->PatchGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->IntensityGapH < b->IntensityGapH && b->IntensityGapH > this->EdgeIntensityGapThreshold)
                                 return true;
                               if (b->IntensityGapH < a->IntensityGapH && a->IntensityGapH > this->EdgeIntensityGapThreshold)
                                 return false;
                               return true;
                             });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->ClearGrid(this->VoxGrid);
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::INTENSITY_EDGE,
                             this->VoxGrid,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->IntensityGapH < b->IntensityGapH && b->IntensityGapH > this->EdgeIntensityGapThreshold)
                                 return true;
                               if (b->IntensityGapH < a->IntensityGapH && a->IntensityGapH > this->EdgeIntensityGapThreshold)
                                 return false;
                               return true;
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeBlobs()
{
  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (const auto& point : *this->Scan)
  {
    // Random sampling to decrease keypoints extraction
    // computation time
    if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
      continue;

    this->AddKeypoint(Keypoint::BLOB, point);
  }
}
} // end of LidarSlam namespace