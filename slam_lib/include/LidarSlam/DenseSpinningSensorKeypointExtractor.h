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

struct PtFeat
{
  int Index;
  float Depth;
  float SpaceGapH;
  float DepthGapH;
  float IntensityGapH;
  float Angle;
  Keypoint KptType;

  PtFeat() : Index(0), Depth(0.0f), SpaceGapH(-1.0f), DepthGapH(-1.0f), IntensityGapH(-1.0f), Angle(1.0f), KptType(UNDEFINED) {}
};

struct IdxVM
{
  int Row;
  int Col;
};
class DenseSpinningSensorKeypointExtractor : public KeypointExtractor
{
public:

  GetMacro(PlaneCosAngleThreshold, float)
  SetMacro(PlaneCosAngleThreshold, float)

  GetMacro(EdgeCosAngleThreshold, float)
  SetMacro(EdgeCosAngleThreshold, float)

  GetMacro(PatchSize, int)
  SetMacro(PatchSize, int)

  GetMacro(VoxelDim, float)
  SetMacro(VoxelDim, float)

  void SetSamplingDSSKE(Keypoint k, SamplingModeDSSKE samplingMode) {this->SamplingDSSKE[k] = samplingMode;};

  SamplingModeDSSKE GetSamplingDSSKE(Keypoint k) {return this->SamplingDSSKE[k];};

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc) override;

  PointCloud::Ptr GetKeypoints(Keypoint k) override;

private:
  // Find the pointer to PtFeat of a point in the scan (designated by its index in the scan)
  // (PtFeat containing the features of the point : space gap, depth...)
  std::shared_ptr<PtFeat> GetPtFeat(int idxInScan);

  // Count the number of non null ptr in a scanline
  int GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine);

  // Initialize LaserIdMap, NbLaserRings, AzimuthalResolution and Pc2VmIndices
  void InitInternalParameters();

  // Auto estimate azimuth angle resolution
  // WARNING: to be correct, the points need to be in the LIDAR sensor
  // coordinates system, where the sensor is spinning around Z axis.
  void EstimateAzimuthalResolution() override;

  // Create vertex map from input pointcloud using indices stored in Pc2VmIndices
  void CreateVertexMap();

  // Clear pointers to PtFeat in the vertex map
  // and the vector of indices Pc2VmIndices
  void ClearVertexMap();

  // Clear Keypoints Poinclouds, reserve new size
  void ClearKeypoints();

  // Output separate point features contained in Vertex Map in pgm format to visualize as 2D image
  void OutputFeatures();

  // Output Keypoints in csv format to use as 3D pointcloud
  // and in pgm format to visualize as 2D image
  bool OutputKeypoints();

  // Compute the curvature features within each scan line : depth
  // space gap, intensity gap and line angle
  void ComputeCurvature() override;

  // Labelize points (unvalid, edge, plane, blob)
  // and extract them in correspondant pointcloud
  void ComputePlanes() override;
  void ComputeEdges() override;
  void ComputeIntensityEdges() override;
  void ComputeBlobs() override;

  // Add point to the keypoint pointcloud
  void AddKeypoint(const Keypoint& k, const LidarPoint &pt);

  // Create square division of the image using 2 dimensions
  void Create2DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid);

  // Create cubic division of the pointcloud using 3 dimensions
  void Create3DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid);

  // Clear patch grid and resets the number of points in the grid
  // To be called at each keypoint computation (in ComputeEdges, ComputePlanes, etc.))
  void ClearGrid(std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>>& grid);

  // Add keypoints of type k to a keypoint pointcloud
  // Using patches to have a uniform distribution of keypoints
  void AddKptsUsingGrid(Keypoint k,
                        std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>> &grid,
                        std::function<bool(const std::shared_ptr<PtFeat>&,
                                           const std::shared_ptr<PtFeat>&)> comparePtFeats);

  // ---------------------------------------------------------------------------
  //   Parameters specific to the DenseSpinningSensorKeypointExtractor
  // ---------------------------------------------------------------------------

  // Sharpness threshold to select a planar keypoint
  // Also used, with its opposite value, to filter too sharp edges
  float PlaneCosAngleThreshold = -0.86;  // ~cos(150°) (selected if cos angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  float EdgeCosAngleThreshold = -0.5; // ~cos(120°) (selected, if cos angle is more than threshold)

  // Downsampling mode
  std::unordered_map<Keypoint, LidarSlam::SamplingModeDSSKE> SamplingDSSKE = {
    {EDGE, LidarSlam::SamplingModeDSSKE::PATCH},
    {PLANE, LidarSlam::SamplingModeDSSKE::PATCH},
    {INTENSITY_EDGE, LidarSlam::SamplingModeDSSKE::PATCH}};

  // Size of a patch (nb of points in one dimension, a patch is a square)
  // Patches are used for 2D grid construction to downsample the keypoints
  // A patch with size 32 means that the patch will contain at most 32x32 points
  int PatchSize = 32; // [nb]

  // Size in meters of a voxel, useful for 3D grid construction
  float VoxelDim = 5.f; // [m]

  // ---------------------------------------------------------------------------
  //   Internal variables specific to the DenseSpinningSensorKeypointExtractor
  // ---------------------------------------------------------------------------

  // Dimensions of the Vertex Map
  int HeightVM;
  int WidthVM;

  // Rotation sense of the lidar
  bool RotationIsClockwise;

  // Map of laser_id to fit random laser_ids into {0, ..., NbLaserRings-1}
  std::unordered_map<int, int> LaserIdMap;

  // Vector linking the index of a point in the pointcloud to its index in the Vertex Map
  std::vector<IdxVM> Pc2VmIndices;

  // Vertex Map of points' indices in the pointcloud
  std::vector<std::vector<std::shared_ptr<PtFeat>>> VertexMap;

  // 3D grid to detect planes far away from the sensor
  std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>> VoxGrid;

  // Patch grid used to downsample the keypoints to reduce global computation time
  std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>> PatchGrid;

  // Struct to store the number of points in each voxel/patch of the used grid;
  int NbPointsInGrid;

  // Extracted keypoints of current frame
  std::map<Keypoint, std::vector<LidarPoint>> Keypoints;
};

} // namespace LidarSlam