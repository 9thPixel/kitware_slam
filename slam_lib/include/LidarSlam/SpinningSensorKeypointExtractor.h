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

#ifndef SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H
#define SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H

#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"

#include <pcl/point_cloud.h>

#include <unordered_map>
#include <bitset>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

class SpinningSensorKeypointExtractor
{
public:
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;

  GetMacro(NbThreads, int)
  SetMacro(NbThreads, int)

  GetMacro(NeighborWidth, int)
  SetMacro(NeighborWidth, int)

  GetMacro(MinDistanceToSensor, double)
  SetMacro(MinDistanceToSensor, double)

  GetMacro(AngleResolution, double)
  SetMacro(AngleResolution, double)

  GetMacro(PlaneSinAngleThreshold, double)
  SetMacro(PlaneSinAngleThreshold, double)

  GetMacro(EdgeSinAngleThreshold, double)
  SetMacro(EdgeSinAngleThreshold, double)

  GetMacro(EdgeDepthGapThreshold, double)
  SetMacro(EdgeDepthGapThreshold, double)

  GetMacro(EdgeSaliencyThreshold, double)
  SetMacro(EdgeSaliencyThreshold, double)

  GetMacro(EdgeIntensityGapThreshold, double)
  SetMacro(EdgeIntensityGapThreshold, double)

  GetMacro(NLasers, int)

  PointCloud::Ptr GetEdgePoints() const { return this->EdgesPoints; }
  PointCloud::Ptr GetPlanarPoints() const { return this->PlanarsPoints; }
  PointCloud::Ptr GetBlobPoints() const { return this->BlobsPoints; }

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature
  void ComputeKeyPoints(const PointCloud::Ptr& pc, const std::vector<size_t>& laserIdMapping);

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<double>> GetDebugArray() const;

private:

  // Reset all mumbers variables that are
  // used during the process of a frame.
  void PrepareDataForNextFrame();

  // Convert the input vtk-format pointcloud
  // into a pcl-pointcloud format. scan lines
  // will also be sorted by their vertical angles
  void ConvertAndSortScanLines();

  // Compute the curvature of the scan lines
  // The curvature is not the one of the surface
  // that intersected the lines but the curvature
  // of the scan lines taken in an isolated way
  void ComputeCurvature();

  // Invalid the points with bad criteria from
  // the list of possible future keypoints.
  // This points correspond to planar surface
  // roughtly parallel to laser beam and points
  // close to a gap created by occlusion
  void InvalidPointWithBadCriteria();

  // Labelizes point to be a keypoints or not
  void SetKeyPointsLabels();

  // Check if scanLine is almost empty
  inline bool IsScanLineAlmostEmpty(int nScanLinePts) const { return nScanLinePts < 2 * this->NeighborWidth + 1; }

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Max number of threads to use to process points in parallel
  int NbThreads = 1;

  // Width of the neighborhood used to compute discrete differential operators
  int NeighborWidth = 4;

  // Minimal point/sensor sensor to consider a point as valid
  double MinDistanceToSensor = 3.0;  // [m]

  // Maximal angle resolution of the lidar azimutal resolution.
  // (default value to VLP-16. We add an extra 20%)
  double AngleResolution = DEG2RAD(0.4);  // [rad]

  // Sharpness threshold to select a planar keypoint
  double PlaneSinAngleThreshold = 0.5;  // sin(30°) (selected if sin angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  double EdgeSinAngleThreshold = 0.86;  // ~sin(60°) (selected, if sin angle is more than threshold)
  double DistToLineThreshold = 0.20;  // [m]

  // Threshold upon depth gap in neighborhood to select an edge keypoint
  double EdgeDepthGapThreshold = 0.15;  // [m]

  // Threshold upon saliency of a neighborhood to select an edge keypoint
  double EdgeSaliencyThreshold = 1.5;  // [m]

  // Threshold upon intensity gap to select an edge keypoint
  double EdgeIntensityGapThreshold = 50.;

  // Threshold upon sphericity of a neighborhood to select a blob point
  double SphericityThreshold = 0.35;  // CHECK : unused

  // Coef to apply to the incertitude radius of the blob neighborhood
  double IncertitudeCoef = 3.0;  // CHECK : unused

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  // Mapping of the lasers id
  std::vector<size_t> LaserIdMapping;

  // Number of lasers scan lines composing the pointcloud
  unsigned int NLasers = 0;

  //! Label of a point as a keypoint
  //! We use binary flags as each point can have different keypoint labels.
  using KeypointFlags = std::bitset<Keypoint::nKeypointTypes>;

  // Curvature and other differential operations (scan by scan, point by point)
  std::vector<std::vector<double>> Angles;
  std::vector<std::vector<double>> DepthGap;
  std::vector<std::vector<double>> Saliency;
  std::vector<std::vector<double>> IntensityGap;
  std::vector<std::vector<KeypointFlags>> IsPointValid;
  std::vector<std::vector<KeypointFlags>> Label;

  // Extracted keypoints of current frame
  PointCloud::Ptr EdgesPoints;
  PointCloud::Ptr PlanarsPoints;
  PointCloud::Ptr BlobsPoints;

  // Current point cloud stored in two differents formats
  PointCloud::Ptr pclCurrentFrame;
  std::vector<PointCloud::Ptr> pclCurrentFrameByScan;
};

#endif // SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H